use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::{
    env, fs,
    io::Write,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::Mutex,
    time::{SystemTime, UNIX_EPOCH},
};
use tauri::ipc::{InvokeBody, Request, Response};

#[derive(Clone)]
struct PythonCommand {
    program: String,
    prefix_args: Vec<String>,
}

#[derive(Clone)]
struct SessionState {
    session_dir: PathBuf,
    scan_path: PathBuf,
    generated: bool,
}

struct WorkstationState {
    planner_root: PathBuf,
    backend_script: PathBuf,
    python: PythonCommand,
    session: Mutex<Option<SessionState>>,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct BackendInfo {
    planner_root: String,
    log_root: String,
    backend_script: String,
    python_command: String,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct ScanInfo {
    file_name: String,
    byte_count: usize,
    session_directory: String,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct GenerationSummary {
    row_count: usize,
    point_count: usize,
    control_point_count: usize,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct GenerateRequest {
    parameters: Value,
    paint_dir: Vec<f64>,
    scan_dir: Vec<f64>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct RegenerateRequest {
    control_points: Value,
    #[serde(default)]
    deleted_row_indices: Vec<usize>,
    #[serde(default)]
    deleted_point_ids: Vec<String>,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct CompletionSummary {
    completed_at: String,
    scan_file: String,
    trajectory_file: String,
    output_directory: String,
}

fn resolve_planner_root() -> Result<PathBuf, String> {
    let mut candidates = Vec::new();
    if let Some(configured) = env::var_os("PAINTING_TRAJECTORY_PLANNER_ROOT") {
        candidates.push(PathBuf::from(configured));
    }
    if let Ok(current_dir) = env::current_dir() {
        candidates.push(current_dir.join("painting_trajectory_planner/path_planner"));
        candidates.push(current_dir.join("../painting_trajectory_planner/path_planner"));
    }
    candidates.push(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join("painting_trajectory_planner/path_planner"),
    );

    for candidate in candidates {
        let normalized = candidate.canonicalize().unwrap_or(candidate);
        if normalized
            .join("scripts/painting_trajectory_planner.py")
            .is_file()
            && normalized.join("config/painting_trajectory.yaml").is_file()
        {
            return Ok(normalized);
        }
    }
    Err(
        "경로 생성기를 찾지 못했습니다. PAINTING_TRAJECTORY_PLANNER_ROOT에 path_planner 경로를 지정하세요."
            .to_string(),
    )
}

fn resolve_backend_script() -> Result<PathBuf, String> {
    let mut candidates = Vec::new();
    if let Some(configured) = env::var_os("WORKSTATION_BACKEND_SCRIPT") {
        candidates.push(PathBuf::from(configured));
    }
    if let Ok(current_dir) = env::current_dir() {
        candidates.push(current_dir.join("backend/workstation_backend.py"));
        candidates.push(current_dir.join("workstation/backend/workstation_backend.py"));
    }
    candidates.push(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("backend/workstation_backend.py"),
    );

    for candidate in candidates {
        let normalized = candidate.canonicalize().unwrap_or(candidate);
        if normalized.is_file() {
            return Ok(normalized);
        }
    }
    Err(
        "workstation_backend.py를 찾지 못했습니다. WORKSTATION_BACKEND_SCRIPT를 지정하세요."
            .to_string(),
    )
}

fn command_available(program: &str, prefix_args: &[String]) -> bool {
    let mut command = Command::new(program);
    configure_external_python(&mut command);
    command
        .args(prefix_args)
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|status| status.success())
}

fn configure_external_python(command: &mut Command) {
    // linuxdeploy's AppRun sets these to paths inside the temporary AppImage
    // mount. They are correct for bundled helpers, but make a system Python
    // treat the AppImage as its own installation and fail before importing
    // the workstation adapter.
    command.env_remove("PYTHONHOME");
    command.env_remove("PYTHONPATH");

    let Some(app_dir) = env::var_os("APPDIR").map(PathBuf::from) else {
        return;
    };
    let Some(library_path) = env::var_os("LD_LIBRARY_PATH") else {
        return;
    };
    let external_paths = env::split_paths(&library_path)
        .filter(|path| !path.starts_with(&app_dir))
        .collect::<Vec<_>>();
    if external_paths.is_empty() {
        command.env_remove("LD_LIBRARY_PATH");
    } else if let Ok(joined) = env::join_paths(external_paths) {
        command.env("LD_LIBRARY_PATH", joined);
    }
}

fn resolve_python() -> Result<PythonCommand, String> {
    if let Some(configured) = env::var_os("PAINTING_WORKSTATION_PYTHON") {
        let command = PythonCommand {
            program: configured.to_string_lossy().into_owned(),
            prefix_args: Vec::new(),
        };
        if command_available(&command.program, &command.prefix_args) {
            return Ok(command);
        }
        return Err(format!(
            "PAINTING_WORKSTATION_PYTHON을 실행하지 못했습니다: {}",
            command.program
        ));
    }

    let candidates = [
        PythonCommand {
            program: "python3".to_string(),
            prefix_args: Vec::new(),
        },
        PythonCommand {
            program: "python".to_string(),
            prefix_args: Vec::new(),
        },
        PythonCommand {
            program: "py".to_string(),
            prefix_args: vec!["-3".to_string()],
        },
    ];
    candidates
        .into_iter()
        .find(|candidate| command_available(&candidate.program, &candidate.prefix_args))
        .ok_or_else(|| {
            "Python 3를 찾지 못했습니다. PAINTING_WORKSTATION_PYTHON을 지정하세요.".to_string()
        })
}

fn percent_decode(value: &str) -> Result<String, String> {
    let bytes = value.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        if bytes[index] == b'%' {
            if index + 2 >= bytes.len() {
                return Err("파일 이름 인코딩이 올바르지 않습니다.".to_string());
            }
            let digits = std::str::from_utf8(&bytes[index + 1..index + 3])
                .map_err(|_| "파일 이름 인코딩이 올바르지 않습니다.".to_string())?;
            decoded.push(
                u8::from_str_radix(digits, 16)
                    .map_err(|_| "파일 이름 인코딩이 올바르지 않습니다.".to_string())?,
            );
            index += 3;
        } else {
            decoded.push(bytes[index]);
            index += 1;
        }
    }
    String::from_utf8(decoded).map_err(|_| "파일 이름이 UTF-8이 아닙니다.".to_string())
}

fn safe_scan_file_name(source: &str) -> Result<String, String> {
    let file_name = Path::new(source)
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or_else(|| "스캔 파일 이름이 올바르지 않습니다.".to_string())?;
    let suffix = Path::new(file_name)
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if suffix != "ply" && suffix != "csv" {
        return Err("스캔 데이터는 PLY 또는 CSV 파일이어야 합니다.".to_string());
    }
    let sanitized = file_name
        .chars()
        .map(|character| {
            if character.is_alphanumeric() || matches!(character, '.' | '-' | '_') {
                character
            } else {
                '_'
            }
        })
        .collect::<String>();
    if sanitized.is_empty() {
        return Err("스캔 파일 이름이 비어 있습니다.".to_string());
    }
    Ok(sanitized)
}

fn lock_session(
    state: &tauri::State<'_, WorkstationState>,
    require_generated: bool,
) -> Result<SessionState, String> {
    let session = state
        .session
        .lock()
        .map_err(|_| "작업 세션 잠금이 손상되었습니다.".to_string())?
        .clone()
        .ok_or_else(|| "먼저 스캔 데이터를 불러오세요.".to_string())?;
    if require_generated && !session.generated {
        return Err("먼저 도장 경로를 생성하세요.".to_string());
    }
    Ok(session)
}

fn run_backend(
    python: &PythonCommand,
    backend_script: &Path,
    action: &str,
    payload: &Value,
) -> Result<Value, String> {
    let mut command = Command::new(&python.program);
    configure_external_python(&mut command);
    let mut child = command
        .args(&python.prefix_args)
        .arg(backend_script)
        .arg(action)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("Python 백엔드를 실행하지 못했습니다: {error}"))?;

    if let Some(mut stdin) = child.stdin.take() {
        let request_bytes = serde_json::to_vec(payload)
            .map_err(|error| format!("백엔드 요청을 직렬화하지 못했습니다: {error}"))?;
        stdin
            .write_all(&request_bytes)
            .map_err(|error| format!("백엔드 요청을 전달하지 못했습니다: {error}"))?;
    }
    let output = child
        .wait_with_output()
        .map_err(|error| format!("Python 백엔드 완료를 기다리지 못했습니다: {error}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr).trim().to_string();
        return Err(if stderr.is_empty() {
            format!(
                "Python 백엔드가 오류 코드 {}로 종료되었습니다.",
                output.status
            )
        } else {
            stderr
        });
    }
    serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("Python 백엔드 응답을 읽지 못했습니다: {error}"))
}

#[tauri::command]
fn get_backend_info(state: tauri::State<'_, WorkstationState>) -> BackendInfo {
    BackendInfo {
        planner_root: state.planner_root.display().to_string(),
        log_root: state.planner_root.join("log").display().to_string(),
        backend_script: state.backend_script.display().to_string(),
        python_command: std::iter::once(state.python.program.as_str())
            .chain(state.python.prefix_args.iter().map(String::as_str))
            .collect::<Vec<_>>()
            .join(" "),
    }
}

#[tauri::command]
fn read_default_parameters(state: tauri::State<'_, WorkstationState>) -> Result<String, String> {
    let path = state.planner_root.join("config/painting_trajectory.yaml");
    fs::read_to_string(&path)
        .map_err(|error| format!("{} 파일을 읽지 못했습니다: {error}", path.display()))
}

#[tauri::command]
fn stage_scan_file(
    request: Request<'_>,
    state: tauri::State<'_, WorkstationState>,
) -> Result<ScanInfo, String> {
    let encoded_name = request
        .headers()
        .get("x-file-name")
        .and_then(|value| value.to_str().ok())
        .ok_or_else(|| "스캔 파일 이름이 전달되지 않았습니다.".to_string())?;
    let file_name = safe_scan_file_name(&percent_decode(encoded_name)?)?;
    let bytes = match request.body() {
        InvokeBody::Raw(bytes) => bytes,
        InvokeBody::Json(_) => {
            return Err("스캔 파일은 바이너리 방식으로 전달해야 합니다.".to_string())
        }
    };
    if bytes.is_empty() {
        return Err("스캔 파일이 비어 있습니다.".to_string());
    }

    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| format!("작업 시간을 만들지 못했습니다: {error}"))?
        .as_nanos();
    let session_dir = env::temp_dir()
        .join("painting_process_workstation")
        .join(format!("{}-{unique}", std::process::id()));
    fs::create_dir_all(&session_dir)
        .map_err(|error| format!("작업 세션을 만들지 못했습니다: {error}"))?;
    let scan_path = session_dir.join(&file_name);
    fs::write(&scan_path, bytes)
        .map_err(|error| format!("스캔 파일을 작업 세션에 저장하지 못했습니다: {error}"))?;

    let mut session = state
        .session
        .lock()
        .map_err(|_| "작업 세션 잠금이 손상되었습니다.".to_string())?;
    *session = Some(SessionState {
        session_dir: session_dir.clone(),
        scan_path,
        generated: false,
    });
    Ok(ScanInfo {
        file_name,
        byte_count: bytes.len(),
        session_directory: session_dir.display().to_string(),
    })
}

#[tauri::command]
async fn generate_trajectory(
    request: GenerateRequest,
    state: tauri::State<'_, WorkstationState>,
) -> Result<GenerationSummary, String> {
    let session = lock_session(&state, false)?;
    let planner_root = state.planner_root.clone();
    let backend_script = state.backend_script.clone();
    let python = state.python.clone();
    let session_dir = session.session_dir.clone();
    let payload = json!({
        "plannerRoot": planner_root,
        "sessionDir": session_dir,
        "scanPath": session.scan_path,
        "parameters": request.parameters,
        "paintDir": request.paint_dir,
        "scanDir": request.scan_dir,
    });
    let response = tauri::async_runtime::spawn_blocking(move || {
        run_backend(&python, &backend_script, "generate", &payload)
    })
    .await
    .map_err(|error| format!("경로 생성 작업이 중단되었습니다: {error}"))??;
    let summary: GenerationSummary = serde_json::from_value(response)
        .map_err(|error| format!("경로 생성 결과가 올바르지 않습니다: {error}"))?;

    let mut current = state
        .session
        .lock()
        .map_err(|_| "작업 세션 잠금이 손상되었습니다.".to_string())?;
    if let Some(current_session) = current.as_mut() {
        if current_session.session_dir == session_dir {
            current_session.generated = true;
        }
    }
    Ok(summary)
}

#[tauri::command]
fn read_generated_trajectory(
    state: tauri::State<'_, WorkstationState>,
) -> Result<Response, String> {
    let session = lock_session(&state, true)?;
    let path = session
        .session_dir
        .join("generated/painting_trajectory.csv");
    let bytes = fs::read(&path)
        .map_err(|error| format!("{} 파일을 읽지 못했습니다: {error}", path.display()))?;
    Ok(Response::new(bytes))
}

#[tauri::command]
fn read_control_points(state: tauri::State<'_, WorkstationState>) -> Result<Value, String> {
    let session = lock_session(&state, true)?;
    let path = session.session_dir.join("edited_control_points.json");
    let source = fs::read_to_string(&path)
        .map_err(|error| format!("{} 파일을 읽지 못했습니다: {error}", path.display()))?;
    serde_json::from_str(&source).map_err(|error| format!("제어점 파일을 읽지 못했습니다: {error}"))
}

#[tauri::command]
async fn regenerate_trajectory(
    request: RegenerateRequest,
    state: tauri::State<'_, WorkstationState>,
) -> Result<GenerationSummary, String> {
    let session = lock_session(&state, true)?;
    let planner_root = state.planner_root.clone();
    let backend_script = state.backend_script.clone();
    let python = state.python.clone();
    let payload = json!({
        "plannerRoot": planner_root,
        "sessionDir": session.session_dir,
        "controlPoints": request.control_points,
        "deletedRowIndices": request.deleted_row_indices,
        "deletedPointIds": request.deleted_point_ids,
    });
    let response = tauri::async_runtime::spawn_blocking(move || {
        run_backend(&python, &backend_script, "regenerate", &payload)
    })
    .await
    .map_err(|error| format!("경로 재생성 작업이 중단되었습니다: {error}"))??;
    serde_json::from_value(response)
        .map_err(|error| format!("경로 재생성 결과가 올바르지 않습니다: {error}"))
}

#[tauri::command]
async fn complete_trajectory(
    state: tauri::State<'_, WorkstationState>,
) -> Result<CompletionSummary, String> {
    let session = lock_session(&state, true)?;
    let planner_root = state.planner_root.clone();
    let backend_script = state.backend_script.clone();
    let python = state.python.clone();
    let payload = json!({
        "plannerRoot": planner_root,
        "sessionDir": session.session_dir,
        "scanPath": session.scan_path,
    });
    let response = tauri::async_runtime::spawn_blocking(move || {
        run_backend(&python, &backend_script, "complete", &payload)
    })
    .await
    .map_err(|error| format!("완료 저장 작업이 중단되었습니다: {error}"))??;
    serde_json::from_value(response)
        .map_err(|error| format!("완료 저장 결과가 올바르지 않습니다: {error}"))
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let planner_root = resolve_planner_root().unwrap_or_else(|error| panic!("{error}"));
    let backend_script = resolve_backend_script().unwrap_or_else(|error| panic!("{error}"));
    let python = resolve_python().unwrap_or_else(|error| panic!("{error}"));
    tauri::Builder::default()
        .manage(WorkstationState {
            planner_root,
            backend_script,
            python,
            session: Mutex::new(None),
        })
        .invoke_handler(tauri::generate_handler![
            get_backend_info,
            read_default_parameters,
            stage_scan_file,
            generate_trajectory,
            read_generated_trajectory,
            read_control_points,
            regenerate_trajectory,
            complete_trajectory,
        ])
        .run(tauri::generate_context!())
        .expect("Painting Process Workstation 실행 중 오류가 발생했습니다.");
}
