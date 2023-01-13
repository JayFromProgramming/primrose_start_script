import subprocess
import os
import threading


class ProcessTracker:
    # This class is started via multiprocessing.Process

    def __init__(self, process_name, process_command, process_env_vars=None, process_cwd=None):
        self.process_name = process_name
        self.process_command = process_command
        self.process_env_vars = process_env_vars
        self.process_cwd = process_cwd
        self.process_terminal = None
        self.status = "Not Started"
        self.running = False
        self.stdout_last_line = ""
        self.stderr_last_line = ""

        # Start the process
        self.thread = threading.Thread(target=self.start)
        self.thread.start()

    def build_launch_script(self):
        lines = []
        if self.process_env_vars is not None:
            # Merge the setup.bash script and the command into one string to execute
            with open(self.process_env_vars, "r") as file:
                lines = file.readlines()
            lines.append(self.process_command)
        with open(f"launch_{self.process_name}.sh", "w") as file:
            file.writelines(lines)

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

        # Start the process
        self.status = "Running"
        while True:
            # Read the stdout and stderr
            stdout = self.process_terminal.stdout.readline()
            stderr = self.process_terminal.stderr.readline()
            if stdout:
                self.stdout_last_line = stdout.decode("utf-8").strip()
                print(self.stdout_last_line)
            if stderr:
                self.stderr_last_line = stderr.decode("utf-8").strip()
                print(self.stderr_last_line)
            if self.process_terminal.poll() is not None:
                # The process has stopped
                self.status = f"Stopped ({self.process_terminal.poll()})"
                break

    def stop(self):
        self.process_terminal.terminate()
        self.status = "Stopped"
