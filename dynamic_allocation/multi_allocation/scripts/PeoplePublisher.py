import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState
from people_msgs.msg import People, Person


class PeoplePublisher:
    def __init__(self):
        rospy.init_node('people_publisher')
        with open("/home/charles/.gazebo/models/person_walking/model.sdf",
                  "r") as f:
            self.person_xml = f.read()
        self.people = People()
        self.people_publisher = rospy.Publisher('/people', People,
                                                queue_size=10)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.move_person = rospy.ServiceProxy('/gazebo/set_model_state',
                                              SetModelState)
        initial_state = rospy.wait_for_message('/transformed_people',
                                               ModelState)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_person = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                          SpawnModel)
        result = spawn_person('person', self.person_xml, '/tracked_person',
                              initial_state.pose, 'map')
        self.people_subscriber = rospy.Subscriber('/transformed_people',
                                                  ModelState,
                                                  self.people_callback)

    def people_callback(self, msg):
        person = Person()
        person.name = 'person'
        person.position = msg.pose.position
        person.velocity.x = msg.twist.linear.x
        person.velocity.y = msg.twist.linear.y
        person.velocity.z = msg.twist.linear.z
        person.reliability = 1.0
        self.people.header.stamp = rospy.Time.now()
        self.people.header.frame_id = 'map'
        self.people.people = [person]
        self.people_publisher.publish(self.people)
        result = self.move_person(msg)
        if not result.success:
            rospy.logerr('Can NOT move person')


if __name__ == '__main__':
    PeoplePublisher()
    rospy.spin()
