"""Motion planning stages, loaded only when their functions are used."""
from importlib.util import LazyLoader, find_spec, module_from_spec
import sys

# Register both names without importing numerical dependencies. ROS launch files
# only reading configuration must not need Conda's CasADi/Pinocchio packages.
# Keep direct imports such as ``from motion_planner.transition import ...`` valid.
for _name, _file in (("time_profile", "1_time_profile"), ("optimization", "2_optimization"),
                     ("transition", "3_transition"), ("sequence", "4_sequence")):
    _fullname = f"{__name__}.{_file}"
    _spec = find_spec(_fullname)
    _spec.loader = LazyLoader(_spec.loader)
    _module = module_from_spec(_spec)
    sys.modules[_fullname] = _module
    globals()[_name] = _module
    sys.modules[__name__ + "." + _name] = _module
    _spec.loader.exec_module(_module)
