"""취득·정합·클러스터링의 세 공정 모듈. import만으로 장비에 연결하지 않는다."""
from importlib.util import LazyLoader, find_spec, module_from_spec
import sys

# 숫자 파일명은 코드에서 역할 이름으로 접근한다.
_MODULES = (("acquisition", "1_acquisition"),
            ("registration", "2_registration"),
            ("clustering", "3_clustering"))

for _name, _filename in _MODULES:
    _fullname = f"{__name__}.{_filename}"
    _spec = find_spec(_fullname)
    _spec.loader = LazyLoader(_spec.loader)
    _module = module_from_spec(_spec)
    sys.modules[_fullname] = _module
    globals()[_name] = _module
    sys.modules[f"{__name__}.{_name}"] = _module
    _spec.loader.exec_module(_module)

__all__ = [name for name, _ in _MODULES]
