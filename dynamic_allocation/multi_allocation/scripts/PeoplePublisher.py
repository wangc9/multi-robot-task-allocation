import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState


class PeoplePublisher:
    def __init__(self):
        rospy.init_node('people_publisher')
        with open("/home/charles/.gazebo/models/person_walking/model.sdf",
                  "r") as f:
            self.person_xml = f.read()
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
        result = self.move_person(msg)
        if not result.success:
            rospy.logerr('Can NOT move person')


if __name__ == '__main__':
    PeoplePublisher()
    rospy.spin()
