"""TCP 프레임 이름 해석.

도장셀 URDF 에서 TCP 는 스프레이건 **노즐 팁** 이어야 한다 (planner 경로가 표면에서
offset_distance 0.3 m 떨어진 노즐 위치이고, 노즐은 플랜지 link_6 에서 +z 로 0.297 m
앞에 있다). 그런데 link_6 을 TCP 로 두고 풀면 건이 부품 속으로 0.3 m 들어간다.

URDF 마다 있는 프레임이 다르므로("tcp" 는 툴 장착 셀 URDF, "spray_gun" 은 그 전
버전, 6축 단독 a0912.white.urdf 에는 둘 다 없다) 요청이 "auto" 면 아래 순서로
**존재하는 첫 프레임** 을 쓴다. 명시적으로 이름을 주면 그대로 쓴다.
"""

TCP_FRAME_AUTO = "auto"
TCP_FRAME_CANDIDATES = ("tcp", "spray_gun", "link_6")


def resolve_tcp_frame_from_names(requested, frame_names):
    """requested 가 'auto' 면 frame_names 안에 있는 첫 후보를 돌려준다."""
    names = set(frame_names)
    if requested and requested != TCP_FRAME_AUTO:
        if requested not in names:
            raise ValueError(f"URDF 에 TCP 프레임 '{requested}' 이 없다")
        return requested
    for candidate in TCP_FRAME_CANDIDATES:
        if candidate in names:
            return candidate
    raise ValueError(f"URDF 에 TCP 후보 프레임 {TCP_FRAME_CANDIDATES} 이 하나도 없다")


def resolve_tcp_frame(model, requested=TCP_FRAME_AUTO):
    """pinocchio 모델용. (frame_name, frame_id) 를 돌려준다."""
    name = resolve_tcp_frame_from_names(requested, [frame.name for frame in model.frames])
    return name, model.getFrameId(name)
