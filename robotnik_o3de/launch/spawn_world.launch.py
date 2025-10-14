from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _start_game(context, executable, world, gui):
    world = world.perform(context)
    world_path = f"Levels/{world}/{world}.spawnable"
    gui = gui.perform(context)

    extended_args = []
    if gui.lower() != "true":
        extended_args.append("-console-mode")

    cfg_path = "/tmp/game.cfg"
    with open(cfg_path, "w") as f:
        print(f"Writing content {world_path} to {cfg_path}")
        f.write(f"LoadLevel {world_path}\n")

    proc = ExecuteProcess(
        cmd=[executable, f"--console-command-file={cfg_path}", *extended_args],
        name="robotnik_game_launcher",
        output="log",
        shell=False,
        emulate_tty=True,
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=proc, on_exit=[EmitEvent(event=Shutdown())])
    )
    return [proc, shutdown_on_exit]

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("world", default_value="demo", description="world name"))
    ld.add_action(DeclareLaunchArgument("gui", default_value="true", description="enable gui"))

    executable = "/home/robotnik/projects/robotnik_roscon25/build/linux/bin/profile/robotnik_roscon25.GameLauncher"

    ld.add_action(
        OpaqueFunction(
            function=_start_game,
            kwargs={
                "executable": executable,
                "world": LaunchConfiguration("world"),
                "gui": LaunchConfiguration("gui"),
            },
        )
    )
    return ld
