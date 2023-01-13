import subprocess
import os
import threading


class ProcessTracker:
    # This class is started via multiprocessing.Process

    def __init__(self, process_name, process_command, process_env_vars=None, process_cwd=None):
        self.process_name = process_name
        self.process_command = process_command if not process_env_vars else f"source {process_env_vars}; {process_command}"
        self.process_cwd = process_cwd
        self.process_terminal = None
        self.status = "Not Started"
        self.running = False
        self.stdout_last_line = ""
        self.stderr_last_line = ""

        # Start the process
        self.thread = threading.Thread(target=self.start)
        self.thread.start()

    def start(self):
        # Setup a shell with the correct environment variables (setup.bash)
        self.process_terminal = subprocess.Popen(
            args=self.process_command,
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



