#!/usr/bin/env python3

"""
You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    # pkg = get_package_share_directory('example_description')
    # rviz_path = os.path.join(pkg,'config','display.rviz')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', rviz_path],
    #     output='screen'
    # )
    # launch_description.add_action(rviz)
    
    # path_description = os.path.join(pkg,'robot','visual','my-robot.xacro')
    # robot_desc_xml = xacro.process_file(path_description).toxml()
    
    # parameters = [{'robot_description':robot_desc_xml}]
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=parameters
    # )
    # launch_description.add_action(robot_state_publisher)


    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui'
    # )
    # launch_description.add_action(joint_state_publisher_gui)
    
    
    """-----------------------------------------ScheduleNode---------------------------------------"""
    ScheduleNode = Node(
        package='example_description',
        namespace='',
        executable='scheduler.py',
        name='scheduler_node',
        parameters=[
        ]
    )
    launch_description.add_action( ScheduleNode )
    
    
    """----------------------------------------TargetPoseNode----------------------------------------"""
    TargetPoseNode = Node(
        package='example_description',
        namespace='',
        executable='target_pose.py',
        name='target_pose_node',
        parameters=[
        ]
    )
    launch_description.add_action( TargetPoseNode )
    
    
    """-----------------------------------TeleopSubscribeNode---------------------------------------"""
    TeleopSubscribeNode = Node(
        package='example_description',
        namespace='',
        executable='teleop_subscribe.py',
        name='teleop_subscribe_node',
        parameters=[
        ]
    )
    launch_description.add_action( TeleopSubscribeNode )
    
    
    """--------------------------------------ControllerNode------------------------------------------"""
    ControllerNode = Node(
        package='example_description',
        namespace='',
        executable='controller.py',
        name='controller_node',
        parameters=[
        ]
    )
    launch_description.add_action( ControllerNode )
    
    
    """--------------------------------------------------------------------------------"""

    
    
    """--------------------------------------------------------------------------------"""
    """--------------------------------------------------------------------------------"""
    """--------------------------------------------------------------------------------"""
    """--------------------------------------------------------------------------------"""
    return launch_description