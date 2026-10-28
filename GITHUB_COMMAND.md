# Push

저장소 루트에서 실행합니다. `git add -A`는 새 파일뿐 아니라 디렉토리 이동에 따른
옛 경로 삭제도 함께 반영합니다. 커밋 전에 반드시 스테이징 내용을 확인하세요.

```bash
git status
git add -A
git diff --cached --stat
git diff --cached --check
git commit -m "message"
git push origin main
```

# Pull

```bash
git pull origin main
git submodule update --init --recursive
```

# Submodule

```bash
cd /home/oms/vehicle_painting_robot
# 등록된 SDK와 모델 저장소 전체 (해당 저장소 접근 권한/네트워크 필요)
git submodule update --init --recursive

# 두산 SDK만 필요한 경우
git submodule update --init --recursive -- robot_controller/doosan_a0912_controller/API-DRFL
```

모델 저장소는 `scan_planning/test/SAM_test/{sam3,efficientsam3}`와
`scan_planning/test/custom_model_test/Car_Painting`에 등록되어 있습니다.
서브모듈 내부를 수정했다면 해당 저장소에서 별도로 커밋하고 접근 가능한 원격 저장소에
push한 뒤, 상위 저장소에서 바뀐 커밋 참조를 반영해야 합니다.

# 공유하는 결과와 Git에 포함하지 않는 로컬 파일

아래 운영 결과 디렉토리는 **Git에 포함**합니다. 하위 CSV·PLY·HTML·설정 스냅샷과
`*.log` 텍스트 로그도 결과 공유 대상으로 포함합니다.

- `scan_planning/log/`
- `path_planning/log/`
- `motion_planning/log/` (`imported_x7y4z3`의 기존 모션 결과 포함)

그 밖의 테스트/SDK/ROS `log/`, `logs/`, `Log/`는 기본적으로 제외합니다.
`backup/`, 빌드·설치·캐시 디렉토리, AppImage, 로컬 `.env` 파일도 계속 제외하며
파일은 로컬에 그대로 보관합니다. 이 제외 규칙은 공유 로그 안에서도 적용합니다.
`.env.example`, `.env.sample`, 코드·설정·메쉬·테스트 입력과
`scan_planning/src/hand_eye_calibration/results/`의 보정값은 제외하지 않습니다.

제외 대상이지만 이미 추적하던 산출물은 **인덱스에서만 제외**했습니다.
커밋 화면의 삭제 표시가 로컬 파일 삭제를 뜻하지는 않습니다.
과거 커밋의 대용량 파일은 이 작업만으로 이력에서 사라지지 않으며, 이력 재작성은 하지 않았습니다.

새로 clone한 PC에는 커밋한 운영 결과가 포함되지만, 테스트 로그·로컬 백업·빌드 결과·AppImage는 없습니다.
환경 설치와 ROS/제어 라이브러리/앱 빌드를 따로 수행하세요.
스캔 재처리·SAM 테스트가 참조하는 원본 촬영 데이터가 테스트 로그에 있다면
해당 입력 데이터는 별도로 공유해야 합니다. 과거 로그 안의 절대 경로는 당시 실행 기록이며,
현재 설정 경로로 일괄 변경하지 않습니다. `check_restructure.py`의 원본 비교 검사도
로컬 백업이 있어야 실행할 수 있습니다.
