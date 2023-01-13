import subprocess
import os
import threading


class ProcessTracker:
    # This class is started via multiprocessing.Process

    def __init__(self, name, process_name, process_command, process_env_vars=[], process_cwd=None):
        self.name = name
        self.process_name = process_name
        self.process_command = process_command
        self.process_env_vars = process_env_vars
        self.process_cwd = process_cwd
        self.process_terminal = None
        self.pid = None
        self.status = "Not Started"
        self.running = False
        self.stdout_last_line = ""
        self.stderr_last_line = ""

        # Start the process
        self.thread = threading.Thread(target=self.start)
        self.thread.start()

    def build_launch_script(self):
        lines = []
        lines.append("#!/bin/bash")
        lines.append("")
        lines.append("# This script is automatically generated by process_tracker.py")
        # lines.append("CATKIN_SHELL=bash")
        for env_var in self.process_env_vars:
            lines.append(f"source {env_var}")
        lines.append(self.process_command)
        script = "\n".join(lines)
        with open(f"launch_{self.process_name}.sh", "w") as file:
            file.write(script)

    def start(self):
        # Setup a shell with the correct environment variables (setup.bash)

        # Create a launch script
        self.build_launch_script()

        self.process_terminal = subprocess.Popen(
            args=f"bash launch_{self.process_name}.sh",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=self.process_cwd
        )
        self.pid = self.process_terminal.pid
        # Start the process
        self.status = "Running"
        self.running = True
        while True:
            # Read the stdout and stderr
            stdout = self.process_terminal.stdout.readline()
            stderr = self.process_terminal.stderr.readline()
            if stdout == b"" and self.process_terminal.poll() is not None:
                self.stdout_last_line = stdout.decode("utf-8").strip()
                # print(self.stdout_last_line)
            if stderr == b"" and self.process_terminal.poll() is not None:
                self.stderr_last_line = stderr.decode("utf-8").strip()
                # print(self.stderr_last_line)
            if self.process_terminal.poll() is not None:
                # The process has stopped
                self.status = f"Stopped ({self.process_terminal.poll()})"
                self.stdout_last_line = stdout.decode("utf-8").strip()
                self.stderr_last_line = stderr.decode("utf-8").strip()
                # print(f"Process {self.process_name} has stopped with exit code {self.process_terminal.poll()}")
                self.running = False
                break

    def stop(self):
        self.process_terminal.terminate()
        self.status = "Stopped"
