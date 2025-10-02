from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from robotnik_common.launch import AddArgumentParser, ExtendedArgument

def _start_game(context, executable, world):
    world = world.perform(context)
    world_path = f"Levels/{world}/{world}.spawnable"

    cfg_path = "/tmp/game.cfg"
    with open(cfg_path, "w") as f:
        f.write(f"LoadLevel {world_path}\n")

    proc = ExecuteProcess(
        cmd=[executable, f"--console-command-file={cfg_path}"],
        name="robotnik_game_launcher",
        output="screen",
        shell=False,
    )

    # mimic on_exit_shutdown=true
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=proc, on_exit=[EmitEvent(event=Shutdown())])
    )

    return [proc, shutdown_on_exit]


def generate_launch_description():
    ld = LaunchDescription()
    add = AddArgumentParser(ld)

    executable = "/home/robotnik/projects/robotnik_roscon25/build/linux/bin/profile/robotnik_roscon25.GameLauncher"

    add.add_arg(ExtendedArgument(name="world", description="world name", default_value="demo"))
    add.add_arg(ExtendedArgument(name="gui", description="enable gui", default_value="true"))

    params = add.process_arg()  # dict of LaunchConfiguration

    ld.add_action(
        OpaqueFunction(
            function=_start_game,
            kwargs={
                "executable": executable,
                "world": params["world"],
            },
        )
    )

    return ld
