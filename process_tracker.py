import subprocess
import os
import threading
import time


class ProcessTracker:
    # This class is started via multiprocessing.Process

    def __init__(self, name, process_name, process_command, process_env_vars=None, process_depends=None,
                 process_cwd=None, stabilize_time=0, dont_start=False):
        self.name = name
        self.process_name = process_name
        self.process_command = process_command
        self.process_env_vars = process_env_vars if process_env_vars is not None else []
        self.process_cwd = process_cwd
        self.process_terminal = None
        self.stabilize_time = stabilize_time
        self.dont_start = dont_start

        self.pid = None
        self.depends = process_depends
        self.status = "Waiting..." if process_depends is not None else "Ready" if not dont_start else "Disabled"
        self.running = False
        self.starting = False
        self.failed = False
        self.stdout_last_line = ""
        self.stdout_log = []
        self.stderr_last_line = ""
        self.stderr_log = []
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
        if self.running or self.starting or self.dont_start:
            return
        self.thread = threading.Thread(target=self._start)
        self.thread.start()

    def dump_logs(self, stdout, stderr):
        with open(f"logs/{self.process_name}.log", "w") as file:
            file.write(stdout)
            file.write("\n STANDARD ERROR \n")
            file.write(stderr)

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
                # Make log file
                self.dump_logs(self.process_terminal.stdout.read().decode("utf-8"),
                               self.process_terminal.stderr.read().decode("utf-8"))
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

            if stdout:
                self.stdout_last_line = stdout.decode("utf-8").strip()
                self.stdout_log.append(self.stdout_last_line)
            if stderr:
                self.stderr_last_line = stderr.decode("utf-8").strip()
                self.stderr_log.append(self.stderr_last_line)

            if self.process_terminal.poll() is not None:
                # The process has stopped
                self.status = f"Execute Failed: {self.process_terminal.returncode}"
                self.stdout_last_line = stdout.decode("utf-8").strip()
                self.stderr_last_line = stderr.decode("utf-8").strip()
                # print(f"Process {self.process_name} has stopped with exit code {self.process_terminal.poll()}")
                self.running = False
                self.failed = True
                break

        # Dump the logs to a file
        with open(f"logs/{self.process_name}.log", "w") as file:
            file.write("\n".join(self.stdout_log))
            file.write("\n STANDARD ERROR \n")
            file.write("\n".join(self.stderr_log))

    def stop(self):
        self.process_terminal.terminate()
        self.status = "Stopped"
