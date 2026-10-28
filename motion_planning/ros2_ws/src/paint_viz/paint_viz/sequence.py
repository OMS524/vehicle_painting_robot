"""Compatibility entry point; implementation lives in motion_planner.sequence."""
from importlib import import_module
_impl = import_module("motion_planner.sequence")
globals().update({k: v for k, v in vars(_impl).items() if not k.startswith("__")})
if __name__ == "__main__":
    raise SystemExit(_impl.main())
