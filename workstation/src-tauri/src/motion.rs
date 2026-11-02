//! External, offline motion planning. Preview never starts a planner or ROS.
use crate::{configure_external_python, run_backend, PythonCommand, WorkstationState};
use serde_json::{json, Value};
use std::{
    collections::HashSet,
    env, fs,
    io::{Read, Seek, SeekFrom},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::Mutex,
    time::{SystemTime, UNIX_EPOCH},
};
use tauri::ipc::Response;

#[derive(Default)]
pub struct MotionState {
    assets: Mutex<HashSet<PathBuf>>,
    jobs: Mutex<Jobs>,
}

#[derive(Default)]
struct Jobs {
    planning: Option<Job>,
    rviz: Option<Job>,
}

struct Job {
    child: Child,
    directory: PathBuf,
    status: String,
    result: Option<Value>,
    stopped: bool,
}

impl Drop for Job {
    fn drop(&mut self) {
        stop(self);
    }
}

fn stop(job: &mut Job) {
    if job.stopped {
        return;
    }
    // Every owned process has its own group. Never stop unrelated ROS/hardware.
    #[cfg(unix)]
    {
        let _ = Command::new("kill")
            .args(["-KILL", "--", &format!("-{}", job.child.id())])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
    }
    let _ = job.child.kill();
    let _ = job.child.wait();
    job.stopped = true;
    if job.status == "running" {
        job.status = "cancelled".into();
    }
}

fn paths(state: &WorkstationState) -> Result<(PathBuf, PathBuf, PythonCommand), String> {
    let root = env::var_os("PAINTING_MOTION_PLANNER_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| state.planner_root.parent().unwrap().join("motion_planning"));
    let root = root
        .canonicalize()
        .map_err(|e| format!("모션 플래너 경로: {e}"))?;
    if !root.join("scripts/motion_planning.py").is_file() {
        return Err(
            "모션 플래너를 찾지 못했습니다. PAINTING_MOTION_PLANNER_ROOT를 확인하세요.".into(),
        );
    }
    let script = state.backend_script.with_file_name("motion_backend.py");
    let home = env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
    let configured = env::var_os("PAINTING_WORKSTATION_MOTION_PYTHON").map(PathBuf::from);
    let python = configured
        .or_else(|| {
            ["miniforge3", "miniconda3", "anaconda3"]
                .iter()
                .map(|base| {
                    home.join(base)
                        .join("envs/vehicle_painting_robot_motion_planning/bin/python")
                })
                .find(|p| p.is_file())
        })
        .ok_or("모션 Python 환경이 없습니다. PAINTING_WORKSTATION_MOTION_PYTHON을 지정하세요.")?;
    Ok((
        root,
        script,
        PythonCommand {
            program: python.to_string_lossy().into_owned(),
            prefix_args: vec!["-s".into()],
        },
    ))
}

#[tauri::command]
pub async fn get_motion_model(
    state: tauri::State<'_, WorkstationState>,
    motion: tauri::State<'_, MotionState>,
) -> Result<Value, String> {
    let (root, script, python) = paths(&state)?;
    let description = root
        .parent()
        .unwrap()
        .join("description")
        .canonicalize()
        .map_err(|e| e.to_string())?;
    let result = tauri::async_runtime::spawn_blocking(move || {
        run_backend(&python, &script, "model", &json!({"motionRoot": root}))
    })
    .await
    .map_err(|e| e.to_string())??;
    let mut assets = HashSet::new();
    for link in result["links"].as_array().ok_or("모델 링크가 없습니다.")? {
        for visual in link["visuals"].as_array().ok_or("모델 visual 오류")? {
            if let Some(asset) = visual["geometry"]["asset"].as_str() {
                let file = description
                    .join(asset)
                    .canonicalize()
                    .map_err(|e| e.to_string())?;
                if !file.starts_with(&description) {
                    return Err("모델 메시가 description 외부에 있습니다.".into());
                }
                assets.insert(file);
            }
        }
    }
    *motion.assets.lock().map_err(|e| e.to_string())? = assets;
    Ok(result)
}

#[tauri::command]
pub fn read_motion_asset(
    asset: String,
    state: tauri::State<'_, WorkstationState>,
    motion: tauri::State<'_, MotionState>,
) -> Result<Response, String> {
    let (root, _, _) = paths(&state)?;
    let file = root
        .parent()
        .unwrap()
        .join("description")
        .join(asset)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !motion
        .assets
        .lock()
        .map_err(|e| e.to_string())?
        .contains(&file)
    {
        return Err("현재 URDF에 등록되지 않은 메시입니다.".into());
    }
    fs::read(file).map(Response::new).map_err(|e| e.to_string())
}

fn spawn(mut command: Command, directory: PathBuf) -> Result<Job, String> {
    let log = fs::File::create(directory.join("process.log")).map_err(|e| e.to_string())?;
    command
        .stdin(Stdio::null())
        .stdout(log.try_clone().map_err(|e| e.to_string())?)
        .stderr(log);
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
    }
    let child = command
        .spawn()
        .map_err(|e| format!("외부 프로세스 실행 실패: {e}"))?;
    Ok(Job {
        child,
        directory,
        status: "running".into(),
        result: None,
        stopped: false,
    })
}

fn new_directory(root: &Path, prefix: &str) -> Result<PathBuf, String> {
    let id = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| e.to_string())?
        .as_nanos();
    let directory = root.join("log").join(format!("{prefix}_{id}"));
    fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
    Ok(directory)
}

#[tauri::command]
pub fn start_motion_job(
    mut request: Value,
    state: tauri::State<'_, WorkstationState>,
    motion: tauri::State<'_, MotionState>,
) -> Result<Value, String> {
    let (root, script, python) = paths(&state)?;
    let source = PathBuf::from(
        request["sourceDirectory"]
            .as_str()
            .ok_or("완료 경로가 필요합니다.")?,
    )
    .canonicalize()
    .map_err(|e| e.to_string())?;
    let logs = state
        .planner_root
        .join("log")
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !source.starts_with(logs) || !source.join("workstation_job.json").is_file() {
        return Err("3단계에서 완료 및 저장한 경로만 사용할 수 있습니다.".into());
    }
    let mut jobs = motion.jobs.lock().map_err(|e| e.to_string())?;
    if jobs
        .planning
        .as_ref()
        .is_some_and(|j| j.status == "running")
    {
        return Err("모션 계획이 이미 실행 중입니다.".into());
    }
    // Close the previous replay so it cannot be mistaken for this job.
    jobs.rviz = None;
    let directory = new_directory(&root, "workstation")?;
    request["sourceDirectory"] = json!(source);
    request["motionRoot"] = json!(root);
    request["jobDir"] = json!(directory);
    let request_file = directory.join("request.json");
    fs::write(
        &request_file,
        serde_json::to_vec_pretty(&request).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    let mut command = Command::new(&python.program);
    configure_external_python(&mut command);
    command
        .args(&python.prefix_args)
        .arg("-u")
        .arg(script)
        .arg("run")
        .arg(request_file);
    jobs.planning = Some(spawn(command, directory.clone())?);
    Ok(json!({"status": "running", "directory": directory}))
}

fn tail(file: &Path) -> String {
    let Ok(mut stream) = fs::File::open(file) else {
        return String::new();
    };
    let size = stream.metadata().map(|m| m.len()).unwrap_or(0);
    let _ = stream.seek(SeekFrom::Start(size.saturating_sub(24 * 1024)));
    let mut bytes = Vec::new();
    let _ = stream.take(24 * 1024).read_to_end(&mut bytes);
    format!(
        "\n--- {} ---\n{}",
        file.display(),
        String::from_utf8_lossy(&bytes)
    )
}

fn snapshot(job: &mut Job, planning: bool) -> Result<Value, String> {
    if job.status == "running" {
        if let Some(status) = job.child.try_wait().map_err(|e| e.to_string())? {
            job.status = if status.success() {
                "succeeded"
            } else {
                "failed"
            }
            .into();
            if planning && status.success() {
                job.result = fs::read(job.directory.join("result.json"))
                    .ok()
                    .and_then(|b| serde_json::from_slice(&b).ok());
                if job.result.is_none() {
                    job.status = "failed".into();
                }
            }
            // Reap any owned descendants immediately; never signal a stale
            // process-group ID hours later when replacing an old result.
            stop(job);
        }
    }
    let mut log = tail(&job.directory.join("process.log"));
    if planning {
        if let Ok(runs) = fs::read_dir(job.directory.join("results")) {
            for run in runs.flatten() {
                if let Ok(files) = fs::read_dir(run.path().join("logs")) {
                    let mut files: Vec<_> = files.flatten().map(|f| f.path()).collect();
                    files.sort_by_key(|p| p.metadata().and_then(|m| m.modified()).ok());
                    for file in files.iter().rev().take(3).rev() {
                        log.push_str(&tail(file));
                    }
                }
            }
        }
    }
    Ok(json!({"status": job.status, "directory": job.directory, "log": log, "result": job.result}))
}

#[tauri::command]
pub fn motion_job_status(motion: tauri::State<'_, MotionState>) -> Result<Value, String> {
    let mut jobs = motion.jobs.lock().map_err(|e| e.to_string())?;
    let planning = jobs
        .planning
        .as_mut()
        .map(|j| snapshot(j, true))
        .transpose()?;
    let rviz = jobs.rviz.as_mut().map(|j| snapshot(j, false)).transpose()?;
    Ok(json!({"planning": planning, "rviz": rviz}))
}

#[tauri::command]
pub fn stop_motion_process(
    kind: String,
    motion: tauri::State<'_, MotionState>,
) -> Result<(), String> {
    let mut jobs = motion.jobs.lock().map_err(|e| e.to_string())?;
    let job = match kind.as_str() {
        "planning" => &mut jobs.planning,
        "rviz" => &mut jobs.rviz,
        _ => return Err("잘못된 프로세스 종류".into()),
    };
    if let Some(job) = job {
        stop(job);
    }
    Ok(())
}

#[tauri::command]
pub fn open_motion_rviz(
    state: tauri::State<'_, WorkstationState>,
    motion: tauri::State<'_, MotionState>,
) -> Result<(), String> {
    if !cfg!(target_os = "linux") {
        return Err("현재 RViz 연동은 Linux / ROS 2 Humble용입니다.".into());
    }
    let (root, _, python) = paths(&state)?;
    if !root.join("ros2_ws/install/setup.bash").is_file() {
        return Err("ROS 워크스페이스를 먼저 빌드하세요: motion_planning/ros2_ws".into());
    }
    let mut jobs = motion.jobs.lock().map_err(|e| e.to_string())?;
    let job = jobs.planning.as_mut().ok_or("먼저 모션을 생성하세요.")?;
    snapshot(job, true)?;
    if job.status != "succeeded" {
        return Err("완료된 모션 계획이 없습니다.".into());
    }
    let result = job.result.as_ref().ok_or("모션 결과가 없습니다.")?;
    let run = result["runDirectory"]
        .as_str()
        .ok_or("결과 디렉토리 오류")?
        .to_owned();
    let directory = new_directory(&root, "workstation_rviz")?;
    let mut command = Command::new("bash");
    configure_external_python(&mut command); // BEFORE sourcing ROS, which supplies PYTHONPATH.
    let env_dir = Path::new(&python.program)
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    command.env("PAINT_CELL_CONDA_ENV", env_dir);
    if let Some(conda_root) = env_dir.parent().and_then(Path::parent) {
        command.env("CONDA_ROOT", conda_root);
    }
    // Arguments, never interpolated user text. No robot-controller entry point.
    command.args(["--noprofile", "--norc", "-c",
        "source \"$1\" || exit; exec \"$2\" -s \"$3\" --run-dir \"$4\" --part workstation_path --speed 1", "workstation-rviz"])
        .arg(root.join("scripts/activate_ros2.sh")).arg(&python.program)
        .arg(root.join("scripts/visualize_motion.py")).arg(run);
    jobs.rviz = None;
    jobs.rviz = Some(spawn(command, directory)?);
    Ok(())
}

/// Called when closing the application, not only when Rust state is dropped.
pub fn shutdown(state: &MotionState) {
    if let Ok(mut jobs) = state.jobs.lock() {
        jobs.planning = None;
        jobs.rviz = None;
    }
}

#[cfg(all(test, unix))]
mod tests {
    use super::*;

    #[test]
    fn process_failure_keeps_original_stderr() {
        let root = env::temp_dir().join(format!("workstation-rust-test-{}", std::process::id()));
        let directory = new_directory(&root, "failure").unwrap();
        let mut command = Command::new("bash");
        command.args(["-c", "printf 'original backend failure\\n' >&2; exit 7"]);
        let mut job = spawn(command, directory).unwrap();
        job.child.wait().unwrap();
        let value = snapshot(&mut job, true).unwrap();
        assert_eq!(value["status"], "failed");
        assert!(value["log"]
            .as_str()
            .unwrap()
            .contains("original backend failure"));
        assert!(job.stopped);
        drop(job);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn cancellation_reaps_owned_process_and_is_idempotent() {
        let root = env::temp_dir().join(format!("workstation-cancel-test-{}", std::process::id()));
        let directory = new_directory(&root, "cancel").unwrap();
        let mut command = Command::new("bash");
        command.args(["-c", "sleep 30 & wait"]);
        let mut job = spawn(command, directory).unwrap();
        stop(&mut job);
        stop(&mut job);
        assert_eq!(job.status, "cancelled");
        assert!(job.child.try_wait().unwrap().is_some());
        drop(job);
        fs::remove_dir_all(root).unwrap();
    }
}
