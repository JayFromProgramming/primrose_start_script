import subprocess
import os
import threading
import time


class ProcessTracker:
    # This class is started via multiprocessing.Process

    def __init__(self, name, process_name, process_command, process_env_vars=None, process_depends=None,
                 process_cwd=None, stabilize_time=0):
        self.name = name
        self.process_name = process_name
        self.process_command = process_command
        self.process_env_vars = process_env_vars if process_env_vars is not None else []
        self.process_cwd = process_cwd
        self.process_terminal = None
        self.stabilize_time = stabilize_time

        self.pid = None
        self.depends = process_depends
        self.status = "Waiting..." if process_depends is not None else "Ready"
        self.running = False
        self.starting = False
        self.failed = False
        self.stdout_last_line = ""
        self.stderr_last_line = ""
        self.build_launch_script()
        self.thread = None

    def build_launch_script(self):
        lines = []
        lines.append("#!/bin/bash")
        lines.append("")
        lines.append("# This script is automatically generated by process_tracker.py")
        # lines.append("CATKIN_SHELL=bash")
        for env_var in self.process_env_vars:
            lines.append(f"{env_var}")
        lines.append(self.process_command)
        script = "\n".join(lines)
        with open(f"launch_scripts/launch_{self.process_name}.sh", "w") as file:
            file.write(script)

    def start(self):
        if self.running or self.starting:
            return
        self.thread = threading.Thread(target=self._start)
        self.thread.start()

    def _start(self):
        # Setup a shell with the correct environment variables (setup.bash)

        self.process_terminal = subprocess.Popen(
            args=f"bash launch_scripts/launch_{self.process_name}.sh",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=self.process_cwd
        )
        self.pid = self.process_terminal.pid
        # Start the process
        self.status = "Starting..."
        self.starting = True
        start_time = time.time()

        while start_time + self.stabilize_time > time.time():
            # Wait for the process to start
            if self.process_terminal.poll() is not None:
                # Process has stopped
                self.status = f"Launch Failed: {self.process_terminal.returncode}"
                self.running = False
                self.failed = True
                return
            time.sleep(0.1)

        self.status = "Running"
        self.running = True

        while True:
            # Read the stdout and stderr
            # Force flush of the buffers
            self.process_terminal.stdout.flush()
            self.process_terminal.stderr.flush()
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
                self.status = f"Execute Failed: {self.process_terminal.returncode}"
                self.stdout_last_line = stdout.decode("utf-8").strip()
                self.stderr_last_line = stderr.decode("utf-8").strip()
                # print(f"Process {self.process_name} has stopped with exit code {self.process_terminal.poll()}")
                self.running = False
                self.failed = True
                break

    def stop(self):
        self.process_terminal.terminate()
        self.status = "Stopped"
