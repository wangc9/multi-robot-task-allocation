#!/usr/bin/env python3

import actionlib
import rosnode
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from multi_allocation.msg import TaskArray
from multi_allocation.srv import FailTask, FailTaskRequest
from multi_allocation.srv import ProcessTask, ProcessTaskRequest


class Controller:
    def __init__(self):
        self.world_frame = "map"
        rospy.init_node("controller")
        self.own_list = []
        self.robot_list = []
        names = rosnode.get_node_names()
        for name in names:
            if "tb3" in name and "move_base" in name:
                self.robot_list.append(f'/{name.split("/")[1]}')
        self.id = f'/{rospy.get_name().split("/")[1]}'
        self.move_client = actionlib.SimpleActionClient(f'{self.id}/move_base',
                                                        MoveBaseAction)
        self.move_client.wait_for_server()
        self.task_subscriber()
        self.perform_tasks()

    def perform_tasks(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # print("loop is running")
            if len(self.own_list) > 0:
                next_target = self.own_list[0]
                del self.own_list[0]
                rospy.wait_for_service(f'{self.id}/process_task')
                task_service = rospy.ServiceProxy(f'{self.id}/process_task',
                                                  ProcessTask)
                request = ProcessTaskRequest()
                request.task = next_target
                result = task_service(request)
                if result:
                    rospy.loginfo(f'{self.id}: {next_target} has been cleared')
                goal = next_target
                goal.header.stamp = rospy.Time.now()
                action_goal = MoveBaseGoal()
                action_goal.target_pose = goal
                print(action_goal)
                self.move_client.send_goal(action_goal)
                self.move_client.wait_for_result(rospy.Duration.from_sec(30.0))

                if self.move_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                    rospy.wait_for_service(f'{self.id}/fail_task')
                    fail_service = rospy.ServiceProxy(f'{self.id}/fail_task',
                                                      FailTask)
                    fail_request = FailTaskRequest()
                    fail_request.task = next_target
                    result = fail_service(fail_request)

            rate.sleep()

    def register_tasks(self, msg):
        rospy.loginfo(f'{self.id}: received new tasks from parent')
        self.own_list = msg.tasks

    def task_subscriber(self):
        tasks = rospy.Subscriber(f'{self.id}/own_tasks', TaskArray,
                                 self.register_tasks)


if __name__ == '__main__':
    node = Controller()
    node.perform_tasks()
    # rospy.spin()
