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
import argparse
import netifaces

try:
    from rich import print as rprint
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live

    ui_available = True
except ImportError:
    ui_available = False

import process_tracker

from loguru import logger as logging

def get_host_names():
    """
    Gets all the ip addresses that can be bound to
    """
    interfaces = []
    for interface in netifaces.interfaces():
        try:
            if netifaces.AF_INET in netifaces.ifaddresses(interface):
                for link in netifaces.ifaddresses(interface)[netifaces.AF_INET]:
                    if link["addr"] != "":
                        interfaces.append(link["addr"])
        except Exception as e:
            logging.debug(f"Error getting interface {interface}: {e}")
            pass
    return interfaces


class Main:

    def format_command(self, command):
        """Keywords that can be used in the command string
        {address}     - The network address of the robot
        {namespace}   - The namespace of the robot
        {serial_port} - The serial port of the robot
        """
        addresses = "\" \"".join(get_host_names())
        return command.format(
            address=addresses
        )

    def __init__(self, namespace):
        # self.roscore = None
        # self.ros1_bridge = None
        # self.rosbridge_server = None
        # self.obs = None
        # self.robot_nodes = []
        # Execute fuser -k 9090/tcp to kill any processes using port 9090 before starting
        logging.info("Killing any processes using port 9090")
        subprocess.Popen(args="fuser -k 9090/tcp", shell=True)
        self.processes = []
        self.namespace = namespace
        if not self.namespace.no_ui and ui_available:
            self.console = Console()
            threading.Thread(target=self.run).start()
        # Check if a director called "launch_scripts" exists
        if not os.path.isdir("launch_scripts"):
            # If not, create it
            os.mkdir("launch_scripts")

        if not os.path.isdir("logs"):
            os.mkdir("logs")

        logging.info("Starting launch scripts")

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

    def get_process(self, name):
        for process in self.processes:
            if process.process_name == name:
                return process

    def start(self):
        targets = json.load(open("launch.json", "r"))
        # Create a process tracker for each process
        for target, values in targets.items():
            process = process_tracker.ProcessTracker(
                name=values["name"],
                process_name=target,
                process_command=self.format_command(values["command"]),
                process_env_vars=values["additional_commands"],
                process_depends=values["depends"],
                process_cwd=values["cwd"],
                stabilize_time=values["stabilize_time"],
                dont_start=True if self.namespace.dont_launch and target in self.namespace.dont_launch else False,
                restart_on_fail=values["restart_on_fail"] if "restart_on_fail" in values else False,
            )
            self.processes.append(process)

        # Start all processes with no dependencies
        for process in self.processes:
            if not process.depends:
                process.start()

        # Check each process every second to see if it can be started
        while True:
            for process in [process for process in self.processes if not process.running and not process.failed]:
                # Check if all dependencies are met
                if process.depends:
                    can_start = True
                    for dependency in process.depends:
                        depend = self.get_process(dependency)
                        if depend is None:
                            can_start = False
                            self.log_print(f"Failed to start {process.name} because {dependency} is not a valid dependency")
                            break
                        if depend.failed:
                            can_start = False
                            process.failed = True
                            process.status = dependency
                            self.log_print(f"Failed to start {process.name} because {dependency} failed to start")
                            break
                        if not depend.running:
                            can_start = False
                            break
                    if can_start:
                        self.log_print(f"Starting {process.name}")
                        process.start()
            time.sleep(1)

    def log_print(self, text):
        if self.namespace.no_ui or not ui_available:
            print(text)

    def display_status(self):
        # Display the status of all processes in a table

        table_table = Table(show_header=False)
        tables = []

        # Chunk the processes into 2 columns
        chunk_size = 10
        if chunk_size == 0:
            return table_table
        chunks = [self.processes[i:i + chunk_size] for i in range(0, len(self.processes), chunk_size)]
        for chunk in chunks:
            table_table.add_column()
            table = Table(show_header=True, header_style="bold magenta", show_lines=True)
            table.add_column("PID", justify="center", style="cyan", no_wrap=True)
            table.add_column("Process Name", width=25)
            table.add_column("Status", width=19)
            # table.add_column("Last Line"
            for process in chunk:
                table.add_row(
                    str(process.pid),
                    process.name,
                    process.get_status(),
                    # str(process.stderr_last_line) if process.stderr_last_line else str(process.stdout_last_line),
                    style=process.get_color()
                )
            tables.append(table)

        # Add the tables to the table table
        table_table.add_row(*tables)

        return table_table
        # Replace the previous table without clearing the console by just moving the cursor up

    def run(self):
        # Display the status of all processes in a table
        print("Starting UI...")
        try:
            with Live(self.display_status(), refresh_per_second=1) as live:
                while True:
                    live.update(self.display_status())
                    time.sleep(1)
        except Exception as e:
            print(f"UI Failed: {e}")


if __name__ == "__main__":
    # Get the launch command line arguments
    # --dont-launch [target]
    # -no-ui
    print("Starting Launch Script...")
    parser = argparse.ArgumentParser()
    parser.add_argument("--dont-launch", nargs="+", help="Don't launch the specified targets")
    parser.add_argument("--ignore-depends", nargs="+", help="Ignore the dependencies of the specified targets")
    parser.add_argument("-no-ui", action="store_true", help="Don't launch the UI")

    args = parser.parse_args()
    Main(namespace=args)
