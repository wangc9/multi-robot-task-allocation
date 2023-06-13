import math

import rospy
from gazebo_msgs.msg import ModelState
from pedsim_msgs.msg import TrackedPersons
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PeopleTracker:
    def __init__(self):
        rospy.init_node('people_tracker')
        self.people_subscriber = rospy.Subscriber(
            '/pedsim_visualizer/tracked_persons', TrackedPersons,
            self.people_transform)
        self.people_publisher = rospy.Publisher('/transformed_people',
                                                ModelState, queue_size=10)

    def people_transform(self, msg):
        pose = msg.tracks[0].pose.pose
        pose.position.x -= 21.0
        pose.position.y -= 10.0
        (roll, pitch, yaw) = euler_from_quaternion([pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w])
        (x, y, z, w) = quaternion_from_euler(roll, pitch, yaw + 0.5 * math.pi)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        twist = msg.tracks[0].twist.twist
        name = 'person'
        reference = 'map'
        transformed = ModelState()
        transformed.pose = pose
        transformed.twist = twist
        transformed.model_name = name
        transformed.reference_frame = reference
        self.people_publisher.publish(transformed)


if __name__ == '__main__':
    PeopleTracker()

    rospy.spin()
