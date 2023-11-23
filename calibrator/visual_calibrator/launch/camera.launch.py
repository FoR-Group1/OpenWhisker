import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.actions import SetEnvironmentVariable

MASTER_CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")

def generate_launch_description():
    camera = actions.Node(
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        output = "screen",
        parameters = [
            os.path.join(MASTER_CONFIG_DIR, "camera_params-yuv.yaml"),
        ],
    )

    return LaunchDescription([
        camera
    ])

def main(argv):
    ld = generate_launch_description()

    print("Starting introspection of launch description...\n")
    print(LaunchIntrospector().format_launch_description(ld))
    print("Starting launch of launch description...\n")
    
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()

if __name__ == "__main__":
    main(sys.argv)