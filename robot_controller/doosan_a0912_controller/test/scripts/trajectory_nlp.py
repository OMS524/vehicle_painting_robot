#!/usr/bin/env python3
"""Compatibility launcher: optimizer moved to motion_planning/src/motion_planner."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "motion_planning/src"))
from motion_planner import optimization as _impl
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
if __name__ == "__main__":
    try:
        raise SystemExit(_impl.main())
    except KeyboardInterrupt:
        raise SystemExit(130)
