# This script starts all ros nodes and required programs for the robot
# This includes but is not limited to:
# - ros1_bridge
# - roscore
# - rosbridge_server
# - obs (for streaming webcam)
# - all robot nodes
import json
import os
# import threading
# import os
import subprocess
import threading
import time

from rich import print as rprint
from rich.console import Console
from rich.table import Table
from rich.live import Live
import process_tracker


class Main:

    def __init__(self):
        # self.roscore = None
        # self.ros1_bridge = None
        # self.rosbridge_server = None
        # self.obs = None
        # self.robot_nodes = []
        self.processes = []
        self.console = Console()
        threading.Thread(target=self.run).start()
        # Check if a director called "launch_scripts" exists
        if not os.path.isdir("launch_scripts"):
            # If not, create it
            os.mkdir("launch_scripts")
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
        targets = json.load(open("launch.json", "r"))
        # Create a process tracker for each process
        for target, values in targets.items():
            process = process_tracker.ProcessTracker(
                name=values["name"],
                process_name=target,
                process_command=values["command"],
                process_env_vars=values["additional_commands"],
                process_depends=values["depends"],
                process_cwd=values["cwd"],
                stabilize_time=values["stabilize_time"]
            )
            self.processes.append(process)

        # Start all processes with no dependencies
        for process in self.processes:
            if not process.depends:
                process.start()

        # Check each process every second to see if it can be started
        while True:
            for process in self.processes:
                if process.depends:
                    if all([self.processes[self.processes.index(p)].running for p in process.depends]):
                        process.start()
            time.sleep(1)

    def display_status(self):
        # Display the status of all processes in a table
        table = Table(show_header=True, header_style="bold magenta", show_lines=True)
        table.add_column("PID", style="dim", width=12)
        table.add_column("Process Name")
        table.add_column("Status")
        table.add_column("Last Line")
        for process in self.processes:
            color = "green" if process.running else "red"
            table.add_row(
                str(process.pid),
                process.name,
                process.status,
                process.stdout_last_line,
                style=color
            )
        # Replace the previous table without clearing the console by just moving the cursor up
        return table

    def run(self):
        # Display the status of all processes in a table
        with Live(self.display_status(), refresh_per_second=1) as live:
            while True:
                live.update(self.display_status())
                time.sleep(1)


if __name__ == "__main__":
    Main()
