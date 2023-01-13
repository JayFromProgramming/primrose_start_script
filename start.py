# This script starts all ros nodes and required programs for the robot
# This includes but is not limited to:
# - ros1_bridge
# - roscore
# - rosbridge_server
# - obs (for streaming webcam)
# - all robot nodes

# import threading
# import os
import subprocess
import time

from rich import print as rprint
from rich.console import Console
from rich.table import Table
import process_tracker


class Main:

    def __init__(self):
        self.roscore = None
        self.ros1_bridge = None
        self.rosbridge_server = None
        self.obs = None
        self.robot_nodes = []
        self.threads = []
        self.console = Console()
        self.start()

    def parse_env_vars(self, setup_bash_path):
        # Execute the setup.bash file and determine which environment variables are set
        # This is done by executing the setup.bash file and then reading the environment variables

        # Execute the setup.bash file
        process = subprocess.Popen(
            args=f"source {setup_bash_path}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        process.wait()

        # Read the environment variables that were set
        env_vars = {}

    def start(self):
        self.roscore = process_tracker.ProcessTracker(
            process_name="roscore",
            process_command="roscore",
            process_env_vars="/opt/ros/noetic/setup.bash",
            # process_cwd="/home/robot"
        )

        self.run()

    def display_status(self):
        # Display the status of all processes in a table
        table = Table(show_header=True, header_style="bold magenta")
        table.add_column("Process Name")
        table.add_column("Status")
        table.add_column("Last Line")
        table.add_row("roscore", self.roscore.status, self.roscore.stdout_last_line)
        self.console.print(table)

    def run(self):
        while True:
            self.console.clear()
            self.display_status()
            time.sleep(1)


if __name__ == "__main__":
    Main()
