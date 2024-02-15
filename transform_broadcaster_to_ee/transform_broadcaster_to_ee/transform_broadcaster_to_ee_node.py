import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from tf_transformations import quaternion_from_matrix, quaternion_matrix, translation_from_matrix
import tf2_py
import numpy as np

def unpack(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return [x, y, z, w]

class StaticTransformBroadcasterNode(Node):
    '''
    This node publishes a transformation from world to the end-effector so that the pointcloud is in the right position in the world coordinate frame. 
    Please refer to the transform tree in the same directory "frames_2024-02-12_08.47.41.pdf" to ensure that the transform tree is correctly applied.
    In order to view the tree, you need to run "ros2 run tf2_tools view_frames.py"
    '''
    def __init__(self, t_path):
        super().__init__('static_transform_broadcaster_node')
        # We also make use of existing transformation.

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer_rgbtodepth = Buffer()
        self.listener_rgbtodepth = TransformListener(self.tf_buffer_rgbtodepth, self, spin_thread=True)
        self.tf_buffer_depthtobase = Buffer()
        self.listener_depthtobase = TransformListener(self.tf_buffer_depthtobase, self, spin_thread=True)

        try:
            # look for transformation from rgb camera to depth. Pointcloud is in depth frame, we convert that to rgb frame. This should be aleady there when you run ros2 azure kinect driver
            # I believe this code is no longer used.
            transform_rgbtodepth = self.tf_buffer_rgbtodepth.lookup_transform(
                target_frame='depth_camera_link',
                source_frame='rgb_camera_link',
                time=rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=5.0)
            )
            transform_matrix_rgbtodepth = np.identity(4)
            q_rgbtodepth = unpack(transform_rgbtodepth.transform.rotation)
            transform_matrix_rgbtodepth[:3, :3] = quaternion_matrix(q_rgbtodepth)[:3,:3]
            transform_matrix_rgbtodepth[0,3] = transform_rgbtodepth.transform.translation.x
            transform_matrix_rgbtodepth[1,3] = transform_rgbtodepth.transform.translation.y
            transform_matrix_rgbtodepth[2,3] = transform_rgbtodepth.transform.translation.z
            print(transform_matrix_rgbtodepth)
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {str(e)}')

        try:
            # Now look for transformation between depth and camera_base
            transform_depthtobase = self.tf_buffer_depthtobase.lookup_transform(
                target_frame='camera_base',
                source_frame='depth_camera_link',
                time=rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=5.0)
            )
            transform_matrix_depthtobase = np.identity(4)
            q_depthtobase = unpack(transform_depthtobase.transform.rotation)
            transform_matrix_depthtobase[:3, :3] = quaternion_matrix(q_depthtobase)[:3,:3]
            transform_matrix_depthtobase[0,3] = transform_depthtobase.transform.translation.x
            transform_matrix_depthtobase[1,3] = transform_depthtobase.transform.translation.y
            transform_matrix_depthtobase[2,3] = transform_depthtobase.transform.translation.z
            print(transform_matrix_depthtobase)
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {str(e)}')

        # ros2 uses quaternion instead of a matrix..
        rot2 = quaternion_from_matrix(np.identity(4))

        # hardcoded tranformation from linear direct motor to the base of the robot. Note the hardcoded dimension in z direction
        transform_stamped2 = TransformStamped()
        transform_stamped2.header.stamp = rclpy.time.Time().to_msg()
        transform_stamped2.header.frame_id = 'linear_direct_motor_base'
        transform_stamped2.child_frame_id = 'link_base'
        transform_stamped2.transform.translation.x = 0.0
        transform_stamped2.transform.translation.y = 0.0
        transform_stamped2.transform.translation.z = 0.135/2
        transform_stamped2.transform.rotation.x = rot2[0]
        transform_stamped2.transform.rotation.y = rot2[1]
        transform_stamped2.transform.rotation.z = rot2[2]
        transform_stamped2.transform.rotation.w = rot2[3]

        # this is a transformation between camera_base to the end effector "xarm_gripper_base_link"
        # how i ended up with this value is by trial and error in rviz2. This needs to be replaced by camera calibration
        angle_x = np.pi
        angle_y = np.pi/2
        rx = [[1,0,0],[0, np.cos(angle_x), -np.sin(angle_x)],[0, np.sin(angle_x), np.cos(angle_x)]]
        ry = [[np.cos(angle_y), 0, np.sin(angle_y)],[0, 1, 0],[-np.sin(angle_y), 0, np.cos(angle_y)]]

        x_rot = np.identity(4)
        x_rot[:3, :3] = rx
        y_rot = np.identity(4)
        y_rot[:3,:3] = ry
        rot = np.dot(x_rot,y_rot)

        rot3 = quaternion_from_matrix(rot)

        transform_stamped3 = TransformStamped()
        transform_stamped3.header.stamp = rclpy.time.Time().to_msg()
        transform_stamped3.header.frame_id = 'xarm_gripper_base_link'
        transform_stamped3.child_frame_id = 'camera_base'
        transform_stamped3.transform.translation.x = 0.08 # reposition camera_base 
        transform_stamped3.transform.translation.y = 0.0
        transform_stamped3.transform.translation.z = 0.0
        transform_stamped3.transform.rotation.x = rot3[0]
        transform_stamped3.transform.rotation.y = rot3[1]
        transform_stamped3.transform.rotation.z = rot3[2]
        transform_stamped3.transform.rotation.w = rot3[3]

        # Broadcast the static transform
        self.static_broadcaster.sendTransform([transform_stamped3, transform_stamped2])


def main():
    rclpy.init()
    node = StaticTransformBroadcasterNode(t_path)
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
