import typer
from robot_rcs_gr.sdk.server import RobotServer


def main(
    config: str,
    freq: int = typer.Option(500, help="Main loop frequency in hz. defaults to 400hz."),
    debug_interval: int = typer.Option(0, help="Debug loop print interval"),
    verbose: bool = typer.Option(False, help="Print internal debug info"),
):
    if not verbose:
        from robot_rcs.logger.fi_logger import Logger

        Logger().state = Logger().STATE_OFF

    robot = RobotServer(config, freq=freq, debug_print_interval=debug_interval, visualize=False)
    robot.spin()


if __name__ == "__main__":
    typer.run(main)
