import os
from glob import glob

from setuptools import find_packages, setup

package_name = "paint_viz"

setup(
    name=package_name,
    version="0.1.0",
    # motion_planner is a source-tree symlink, not a second copy of the algorithms.
    packages=["paint_viz", "motion_planner"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "gantry"), glob("config/gantry/*.yaml")),
        (os.path.join("share", package_name, "config", "sequence"), glob("config/sequence/*.yaml")),
        (os.path.join("share", package_name, "config", "parts"), glob("config/parts/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf") + glob("urdf/*.stl")),  # STL: package://paint_viz/urdf/… 해석용
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="airo",
    maintainer_email="vontte21@hanyang.ac.kr",
    description="도장 부품 점군, planner TCP 경로, IK 관절 궤적을 RViz2로 시각화한다.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "scene_publisher = paint_viz.scene_publisher:main",
            "trajectory_player = paint_viz.trajectory_player:main",
            "solve_viz_ik = paint_viz.viz_ik:main",
            "solve_dp_ik = paint_viz.dp_ik:main",
            "make_primitive_urdf = paint_viz.make_primitive_urdf:main",
            "make_gantry_urdf = paint_viz.make_gantry_urdf:main",
            "split_gantry_stl = paint_viz.split_gantry_stl:main",
            "make_dock_urdfs = paint_viz.make_dock_urdfs:main",
            "dock_path = paint_viz.dock_path:main",
            "stitch_transitions = paint_viz.stitch:main",
            "check_collisions = paint_viz.collision_check:main",
            "cell_pose_info = paint_viz.cell_pose_info:main",
            "gantry_env = paint_viz.gantry_env:main",
            "build_sequence = paint_viz.sequence:main",
        ],
    },
)
