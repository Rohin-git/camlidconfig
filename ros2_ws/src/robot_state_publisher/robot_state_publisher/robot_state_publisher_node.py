import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi
import urdf_parser_py.urdf as urdf

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self)
        print('Inside state publisher')

        # Load URDF file
        self.urdf_model = urdf.URDF.from_xml_file("/home/rohin/camlid/camlidconfig/ros2_ws/src/my_package/urdf/vehicle.urdf")

        self.loop()

    def loop(self):
        degree = pi / 180.0
        loop_rate = self.create_rate(30)
        print('Inside loop')

        try:
            while rclpy.ok():
                now = self.get_clock().now()

                # Update joint states and broadcast transforms
                for joint_name, joint in self.urdf_model.joints.items():
                    if joint.type == 'fixed':
                        # Replace with actual joint position if available
                        joint_state.name.append(joint_name)
                        joint_state.position.append(0.0)  # Placeholder for actual joint position

                if joint_state.name:
                    self.joint_pub.publish(joint_state)

                for link_name, link in self.urdf_model.links.items():
                    t = TransformStamped()
                    t.header.stamp = now.to_msg()
                    t.header.frame_id = link_name
                    t.child_frame_id = link_name
                    t.transform.translation.x = link.origin.xyz[0]
                    t.transform.translation.y = link.origin.xyz[1]
                    t.transform.translation.z = link.origin.xyz[2]
                    q = Quaternion()
                    q.x, q.y, q.z, q.w = link.origin.rpy
                    t.transform.rotation = q
                    self.broadcaster.sendTransform(t)

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    rclpy.init()
    print('Inside main')
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# def main():
#     print('Hi from robot_state_publisher.')


# if __name__ == '__main__':
#     main()
