import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    intra_process = False
    num_publishers = 60
    num_subscriptions = 60
    string_publish_rate = 1.0

    single_process_nodes = []
    for i in range(num_publishers):
        single_process_nodes.append(
            Node(
                name=f'timer_publish_{i:05}',
                package='test_pkg',
                executable='timer_publish_node',
                remappings=[('~/output', '/timer_publish/output')],
                parameters=[{'publish_rate': string_publish_rate}],
                ros_arguments=['--log-level', 'info'],
            )
        )

    components = []
    for i in range(num_subscriptions):
        components.append(
            ComposableNode(
                name=f'subscribe_{i:05}',
                package='test_pkg',
                plugin='test_pkg::SubscribeComposableNode',
                remappings=[('~/input', '/timer_publish/output')],
                extra_arguments=[{'use_intra_process_comms': intra_process}],
            )
        )

    component_container = ComposableNodeContainer(
        namespace='',
        name='component_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        ros_arguments=['--log-level', 'info'],
        composable_node_descriptions=components,
        output='both'
    )

    return launch.LaunchDescription([
        *single_process_nodes, component_container
    ])
