"""다중/단일 프레임 스캔이 공유하는 정합 TF 선택. 하드웨어를 직접 연결하지 않는다."""
import hashlib
import json
from pathlib import Path

import numpy as np

from cartesian_scan import sdk_pose_matrix, validate_transform

DEPTH_FRAME = "camera_depth_optical_frame"
SDK_POSE_SOURCE = "Doosan SDK measured base->flange, mm/Euler ZYZ converted to m"


def configure_pose_source(cfg, config_path):
    settings = cfg.setdefault("pose_estimation", {})
    if not isinstance(settings, dict):
        raise ValueError("pose_estimation은 source와 hand_eye_calibration_file을 가진 설정이어야 합니다.")
    settings.setdefault("source", "urdf")
    settings.setdefault("hand_eye_calibration_file", None)
    if settings["source"] not in ("urdf", "hand_eye"):
        raise ValueError("pose_estimation.source는 urdf / hand_eye만 가능합니다.")
    value = settings["hand_eye_calibration_file"]
    if value is not None:
        if not isinstance(value, str) or not value.strip():
            raise ValueError("hand_eye_calibration_file은 JSON 파일 경로 또는 null이어야 합니다.")
        path = Path(value).expanduser()
        settings["hand_eye_calibration_file"] = str(
            (Path(config_path).parent / path).resolve() if not path.is_absolute() else path.resolve())


class ScanPoseSource:
    def __init__(self, cfg, model):
        self.source = cfg.get("pose_estimation", {}).get("source", "urdf")
        if self.source not in ("urdf", "hand_eye"):
            raise ValueError("pose_estimation.source는 urdf / hand_eye만 가능합니다.")
        self.model = model
        nominal = model.transforms([0.0] * 6)
        inv_flange = np.linalg.inv(nominal["link_6"])
        self.flange_bracket = validate_transform(inv_flange @ nominal["bracket_link"])
        self.flange_depth = validate_transform(inv_flange @ nominal[DEPTH_FRAME])
        self.metadata = {"source": self.source}
        self.report_bytes = None
        if self.source == "hand_eye":
            filename = cfg["pose_estimation"].get("hand_eye_calibration_file")
            if not filename:
                raise ValueError("hand_eye 모드에는 pose_estimation.hand_eye_calibration_file이 필요합니다.")
            self.report_bytes = Path(filename).read_bytes()
            report = json.loads(self.report_bytes)
            if not isinstance(report, dict):
                raise ValueError("핸드아이 결과는 calibration.json 객체여야 합니다.")
            if (report.get("status") != "computed_requires_physical_validation"
                    or report.get("quality_checks_passed") is not True):
                raise ValueError("핸드아이 계산 완료 및 품질 검사 통과 결과가 필요합니다. 실패/미완료 결과는 사용하지 않습니다.")
            if report.get("translation_unit") != "m" or report.get("robot_pose_source") != SDK_POSE_SOURCE:
                raise ValueError("SDK 실측 플랜지 기준·위치 단위 m인 핸드아이 결과가 필요합니다.")
            if "T_flange_depth" not in report:
                raise ValueError("핸드아이 결과에 T_flange_depth가 없습니다. RGB 결과만으로 Depth TF를 대체하지 않습니다.")
            self.flange_depth = validate_transform(report["T_flange_depth"])
            self.metadata.update(hand_eye_calibration_file=str(filename),
                source_sha256=hashlib.sha256(self.report_bytes).hexdigest(),
                calibration_method=report.get("method"),
                T_flange_depth=self.flange_depth.tolist())

    @property
    def method(self):
        return ("actual_joint_fk_and_xacro_tf_only" if self.source == "urdf"
                else "actual_sdk_flange_and_hand_eye_tf_only")

    @property
    def pose_source(self):
        return ("actual_joint_measurement_after_capture_plus_URDF_FK" if self.source == "urdf"
                else "actual_sdk_flange_after_capture_plus_hand_eye")

    def snapshot(self, robot):
        if self.source == "urdf":
            return robot.snapshot()
        state = robot.call("snapshot_cartesian")
        # 누락/비유한 SDK 자세를 URDF로 대체하지 않고 촬영을 중단한다.
        sdk_pose_matrix(state["flange_pose_mm_zyz_deg"])
        return state

    def frames(self, state):
        if self.source == "urdf":
            return self.model.transforms(state["joint_deg"])
        flange = sdk_pose_matrix(state["flange_pose_mm_zyz_deg"])
        return {"link_6": flange, "bracket_link": flange @ self.flange_bracket,
                DEPTH_FRAME: flange @ self.flange_depth}

    def save_report(self, output):
        if self.report_bytes is not None:
            # 선택한 원본과 동일한 바이트를 보존. 세션 중 원본이 수정돼도 TF는 바뀌지 않는다.
            (Path(output) / "hand_eye_calibration.json").write_bytes(self.report_bytes)

    def describe(self):
        print(f"정합 TF 모드: {self.source}")
        if self.source == "hand_eye":
            print("SDK 실측 base→flange × 보정 T_flange_depth 사용:",
                  self.metadata["hand_eye_calibration_file"])
            print("보정값의 카메라/브라켓 장착 상태가 현재와 같은지 확인하세요. 품질 검사는 실물 정확도 보증이 아닙니다.")
        else:
            print("실측 관절각 + URDF FK/고정 장착 TF 사용")
        print("T_flange_depth (Depth 좌표를 플랜지 좌표로 변환, 위치 m):")
        print(self.flange_depth)
