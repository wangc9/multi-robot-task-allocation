import math
import random

from shapely.geometry import Polygon, Point

import rosnode
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


class EnvironmentController:
    def __init__(self):
        self.world_frame = "map"
        rospy.init_node("door_controller")
        self.robot_list = []
        names = rosnode.get_node_names()
        for name in names:
            if "tb3" in name and "move_base" in name:
                self.robot_list.append(f'/{name.split("/")[1]}')
        with open("/home/charles/.gazebo/models/blue_door/model.sdf", "r") as f:
            self.door_xml = f.read()
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.add_door = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                           SpawnModel)
        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_door_srv = rospy.ServiceProxy('/gazebo/delete_model',
                                                  DeleteModel)
        self.centres = []
        self.door_inflation_poly = []
        self.build_doors()

    def build_doors(self):
        """
        Build all the doors in the simulation so that they are ready to be \
        spawned or deleted.
        :return:
        """
        self.centres = [[-6, -3, True, False, 0], [-6, -1, True, False, 1],
                        [-6, 1, True, False, 2], [-4, -3, True, False, 3],
                        [-4, -1, True, False, 4], [-3, 2, False, False, 5],
                        [-1, 2, False, False, 6], [1, 2, False, False, 7],
                        [4, 2, True, False, 8], [4, -2, True, False, 9],
                        [6, -4, False, False, 10], [8, -2, True, False, 11],
                        [8, 2, True, False, 12], [6, 4, False, False, 13]]
        for entry in self.centres:
            p_1 = (entry[0] - 1.0, entry[0] - 1.0)
            p_2 = (entry[0] - 1.0, entry[0] + 1.0)
            p_3 = (entry[0] + 1.0, entry[0] + 1.0)
            p_4 = (entry[0] + 1.0, entry[0] - 1.0)
            points = [p_1, p_2, p_3, p_4]
            self.door_inflation_poly.append(Polygon(points))

        while not rospy.is_shutdown():
            rate = random.randint(1, 60)
            door_cand = random.randint(0, 13)
            rospy.loginfo(f'Spawning door {door_cand} in {rate} seconds')
            if self.centres[door_cand][-2]:
                self.delete_door(door_cand)
            else:
                self.spawn_doors(self.centres[door_cand])
            rospy.sleep(rate)

    def surface_check(self, door_index: int) -> bool:
        """
        Check whether the door can be changed.

        Check the surroundings of the door. The door should only be changed \
        when there are no robots in its surrounding area.

        :param door_index: Index of the door

        :return: bool (True if nothing is near the area, the door can be \
            changed, otherwise False)
        """
        model_states = rospy.wait_for_message('/gazebo/model_states',
                                              ModelStates)
        model_names = model_states.name
        for robot in self.robot_list:
            position = model_states.pose[model_names.index(robot.split('/')[1])]
            point = Point(position.x, position.y)
            if self.door_inflation_poly[door_index].contains(point):
                return False
        return True

    def spawn_doors(self, condition):
        """
        Spawn door in simulation.

        Spawn the specified door in gazebo. Take in the position and \
        orientation of the door, check its surroundings and spawn.

        :param condition: [x: int, y: int, turn: bool (T if the door needs to \
            be turned 90 degree), spawned: bool, index: int]

        :return: None
        """
        door_pose = Pose()
        door_pose.position.x = condition[0]
        door_pose.position.y = condition[1]
        door_pose.position.z = 1.0
        if condition[2]:
            (x, y, z, w) = quaternion_from_euler(0.0, 0.0, 0.5 * math.pi)
        else:
            (x, y, z, w) = quaternion_from_euler(0.0, 0.0, 0.0)
        door_pose.orientation.x = x
        door_pose.orientation.y = y
        door_pose.orientation.z = z
        door_pose.orientation.w = w
        if self.surface_check(condition[-1]):
            result = self.add_door(f'door_{condition[-1]}', self.door_xml,
                                   '/doors', door_pose, 'world')
            if result.success:
                self.centres[condition[-1]][-2] = True
        else:
            rospy.loginfo(f'door {condition[-1]} did not change')

    def delete_door(self, door_index):
        model_states = rospy.wait_for_message('/gazebo/model_states',
                                              ModelStates)
        model_names = model_states.name
        if f'door_{door_index}' not in model_names:
            rospy.logerr(f'Door {door_index} DOES NOT EXIST')
        else:
            result = self.delete_door_srv(f'door_{door_index}')
            if result.success:
                self.centres[door_index][-2] = False


if __name__ == '__main__':
    EnvironmentController()
