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
# import rich
import process_tracker


class Main:

    def __init__(self):
        self.roscore = None
        self.ros1_bridge = None
        self.rosbridge_server = None
        self.obs = None
        self.robot_nodes = []
        self.threads = []
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
        while True:
            print(self.roscore.status)

if __name__ == "__main__":
    Main()
