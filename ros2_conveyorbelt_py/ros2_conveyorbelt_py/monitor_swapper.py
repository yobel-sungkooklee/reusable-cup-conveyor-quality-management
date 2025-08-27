import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import os

class MonitorSwapper(Node):
    def __init__(self):
        super().__init__('monitor_swapper')

        self.models = ['inspection_monitor_defect', 'inspection_monitor_nodefect']
        self.current = 0

        self.model_dir = os.path.expanduser(
            '~/dev_ws/src/IFRA_ConveyorBelt/conveyorbelt_gazebo/models'
        )

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.timer = self.create_timer(3, self.swap_model) # 3초마다 spawn

    def swap_model(self):
        to_spawn = self.models[self.current]
        to_delete = self.models[(self.current + 1) % 2]
        self.current = (self.current + 1) % 2

        # delete
        if self.delete_cli.service_is_ready():
            del_req = DeleteEntity.Request()
            del_req.name = to_delete
            self.delete_cli.call_async(del_req)

        # spawn
        if self.spawn_cli.service_is_ready():
            sdf_path = os.path.join(self.model_dir, to_spawn, 'model.sdf')
            with open(sdf_path, 'r') as f:
                sdf = f.read()

            spawn_req = SpawnEntity.Request()
            spawn_req.name = to_spawn
            spawn_req.xml = sdf
            spawn_req.robot_namespace = ''
            spawn_req.reference_frame = 'world'
            spawn_req.initial_pose.position.x = -0.3
            spawn_req.initial_pose.position.y = 0.0
            spawn_req.initial_pose.position.z = 1.0

            self.spawn_cli.call_async(spawn_req)

        self.get_logger().info(f'Spawned: {to_spawn}, Deleted: {to_delete}')

def main():
    rclpy.init()
    node = MonitorSwapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
