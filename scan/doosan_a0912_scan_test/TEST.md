# 1. 카메라 입력 테스트
```bash
conda activate vehicle_painting_robot_scan

SDK_DIR="$(python -c 'import importlib.util, os; print(os.path.dirname(importlib.util.find_spec("pyorbbecsdk").origin))')"

cd "$SDK_DIR"
python examples/quick_start.py
```

# 2. xacro visualize
```bash
conda activate vehicle_painting_robot_scan

python /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts/visualize_xacro.py
```

# 3. Scan
```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts

python three_view_scan.py --check
python three_view_scan.py --execute
```
```bash
python three_view_scan.py --view ../log/촬영디렉토리/merged.ply
```
