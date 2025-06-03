# 3-Clause BSD License

# Copyright 2025 Takumi Asada

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import launch
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with multiple components."""

    # Define imu madgwick node
    # imu_madgwick_node = Node(
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',
    #     name='imu_filter',
    #     output='screen',
    #     parameters=[{
    #       'world_frame': 'ned',
    #     }]
    # )

    imu_madgwick_node = Node(
      package='imu_complementary_filter',
      executable='complementary_filter_node',
      name='complementary_filter_gain_node',
      output='screen',
      parameters=[
          {'do_bias_estimation': True},
          {'do_adaptive_gain': True},
          {'use_mag': True},
          {'gain_acc': 0.01},
          {'gain_mag': 0.01},
      ],
    )

    imu_bno055_node = Node(
        package='ros_imu_bno055',
        executable='imu_bno055_node',
        output="screen",
    )

    return launch.LaunchDescription([
        imu_bno055_node,
        imu_madgwick_node
    ])
