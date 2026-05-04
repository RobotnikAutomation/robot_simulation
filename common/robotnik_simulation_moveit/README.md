# MoveIt Bringup for Simulation

This package provides a standalone MoveIt launch flow for Robotnik simulated robots.

It can be used in two ways:
- Launched directly with `moveit.launch.py`
- Included from `robotnik_simulation_bringup` when `run_moveit:=true`

## Launch

### Standalone

```bash
ros2 launch robotnik_simulation_moveit moveit.launch.py \
  robot_id:=robot \
  robot:=rbkairos \
  robot_model:=rbkairos_plus \
  moveit_config_name:=rbkairos_moveit_config \
  arm_type:=ur10e \
  run_moveit_rviz:=true
```

## Launch files

- `launch/moveit.launch.py`: Orchestrator launch. Includes move_group and optional MoveIt RViz.
- `launch/move_group.launch.py`: MoveIt move_group node setup and planning pipelines.
- `launch/moveit_rviz.launch.py`: Manipulation RViz setup.

## Parameters

| Name | Required | Purpose | Example |
|---|---|---|---|
| `robot_id` | no | Unique robot namespace and prefix root | `robot` |
| `robot` | no | Robot type used to resolve robot description paths | `rbkairos` |
| `robot_model` | no | Robot model variant | `rbkairos_plus` |
| `moveit_config_name` | no | MoveIt config package name | `rbkairos_moveit_config` |
| `arm_type` | no | Arm type xacro argument | `ur10e` |
| `use_sim_time` | no | Use simulation clock | `true` |
| `run_moveit_rviz` | no | Enable MoveIt RViz launch | `true` |
| `moveit_rviz_config` | no | RViz config path for manipulation UI | `/path/to/moveit.rviz` |
| `use_fixed_frame` | no | Add RViz fixed frame argument as `<robot_id>_odom` | `false` |

## Notes

- Recommended robot namespace is `robot` for full RViz interaction compatibility.
- This package assumes simulation and controllers are already running.
