import datetime
import math
import pickle
import random
from typing import List

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from multi_allocation.srv import RoomFinding
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler


class TaskGenerator:
    def __init__(self, task_number: int, known_tasks: bool, can_repeat: bool):
        """
        Initiate TaskGenerator class.

        :param task_number: total amount of tasks
        :param known_tasks: True if all tasks are known beforehand
        :param can_repeat: True if tasks can be repeated
        """
        self.map = None
        self.np_map = None
        self.cost_map = None
        self.np_costmap = None
        self.task_number = task_number
        self.known_tasks = known_tasks
        self.can_repeat = can_repeat
        self.tasks = []
        self.generate_tasks()

    def generate_tasks(self):
        """
        Generate tasks

        Generate a list of tasks (PoseStamped) and staged in a pickle file. \
        The number of tasks is decided by `task_number`. If the algorithm to \
        be tested requires known tasks, set `known_tasks` to True. If tasks \
        can be spawn for more than once, set `can_repeat` to True.

        :return: None. Tasks will be saved in a pickle file
        """
        counter = 0
        task_index = []
        rospy.wait_for_service('/tb3_2/room_finding')
        room_finding_srv = rospy.ServiceProxy('/tb3_2/room_finding',
                                              RoomFinding)
        rospy.loginfo(f'Room finding found')
        self.map = rospy.wait_for_message('/tb3_2/map', OccupancyGrid)
        self.np_map = np.array(self.map.data, dtype=np.int8).reshape(384, 832)
        rospy.loginfo(f'Map found')
        self.cost_map = rospy.wait_for_message(
            '/tb3_0/move_base/global_costmap/costmap', OccupancyGrid)
        self.np_costmap = np.array(self.cost_map.data, dtype=np.int8).reshape(
            384, 832)
        rospy.loginfo(f'Costmap found')
        while counter < self.task_number:
            column = random.randint(0, 383)
            row = random.randint(0, 831)
            yaw = random.uniform(-math.pi, math.pi)
            if 1 < self.np_costmap[column, row] < 42 and self.np_map[
                column, row] != -1:
                if not self.can_repeat:
                    if [column, row] in task_index:
                        continue
                task_index.append([column, row])
                world_coordinate = self.grid_to_world([column, row])
                (q_x, q_y, q_z, q_w) = quaternion_from_euler(0.0, 0.0, yaw)
                task = PoseStamped()
                task.header.seq = counter
                task.header.frame_id = 'map'
                task.pose.position.x = world_coordinate[0]
                task.pose.position.y = world_coordinate[1]
                task.pose.position.z = 0.0
                task.pose.orientation.x = q_x
                task.pose.orientation.y = q_y
                task.pose.orientation.z = q_z
                task.pose.orientation.w = q_w
                result = room_finding_srv(task)
                if result:
                    self.tasks.append(task)
                    counter += 1
                    rospy.loginfo(f'{counter} generated')
        task = PoseStamped()
        task.header.seq = counter
        task.header.frame_id = 'map'
        task.pose.position.x = -22.0
        task.pose.position.y = -10.0
        task.pose.position.z = 0.0
        task.pose.orientation.x = 0.0
        task.pose.orientation.y = 0.0
        task.pose.orientation.z = 0.0
        task.pose.orientation.w = 1.0
        self.tasks.append(task)
        current_time = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        task_file_path = f'tasks_{current_time}.pkl'

        with open(task_file_path, 'wb') as f:
            pickle.dump(self.tasks, f)
        print(self.tasks)

    def grid_to_world(self, point: List[int]):
        """
        Calculate world coordinate.

        :param point: grid coordinate of [grid_y, grid_x]

        :return: world coordinate of (world_x, world_y)
        """
        x = point[1] * 0.05000000074505806 - 21.2
        y = point[0] * 0.05000000074505806 - 10.0

        return (x, y)


if __name__ == '__main__':
    rospy.init_node('task_generator')
    task_number = int(rospy.get_param('~task_number', 10))
    known_tasks = True if rospy.get_param('~known_tasks',
                                          False) is True else False
    can_repeat = True if rospy.get_param('~can_repeat',
                                         False) is True else False
    generator = TaskGenerator(task_number, known_tasks, can_repeat)
