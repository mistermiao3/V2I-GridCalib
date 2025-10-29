# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    This function generates the launch description for starting the rasterizer node.
    It defines the package, executable, and name for the node to be launched.

    这个函数生成用于启动光栅化节点的启动描述。
    它定义了要启动的节点所在的包名、可执行文件名和节点名。
    """
    return LaunchDescription([
        # Defines the node to be launched.
        # 定义要启动的节点。
        Node(
            # The name of the package where the node's executable is located.
            # 节点可执行文件所在的包名。
            package='v2i_gridcalib_rasterizer',

            # The name of the executable as defined in the 'console_scripts' entry point in setup.py.
            # 在 setup.py 文件的 'console_scripts' 入口点中定义的可执行文件名。
            executable='rasterizer_node',

            # The name that the node will have in the ROS2 graph when it is running.
            # 当节点运行时，它在ROS2计算图中所拥有的名称。
            name='rasterizer_node',

            # This ensures that the node's log messages (INFO, WARN, ERROR) are printed to the console.
            # 这确保了节点的日志消息（INFO, WARN, ERROR）能够被打印到控制台。
            output='screen',
            
            # This setting helps ensure that output from the Python script is flushed immediately,
            # which is useful for seeing print statements in real-time.
            # 这个设置有助于确保Python脚本的输出被立即刷新，
            # 这对于实时看到 print 语句很有用。
            emulate_tty=True
        )
    ])