#!/usr/bin/env python3

import actionlib
import rosnode
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
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
        self.counter = 0
        self.trigger_1 = False
        self.trigger_2 = False
        self.received = False
        self.next_target = None
        self.cancelled = False
        current_pose = rospy.wait_for_message(f"{self.id}/amcl_pose",
                                              PoseWithCovarianceStamped)
        self.current_pose = PoseStamped()
        self.current_pose.header = current_pose.header
        self.current_pose.pose = current_pose.pose.pose
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
                if self.trigger_1 and len(self.own_list) > 10:
                    self.own_list = self.own_list[:10]
                self.next_target = self.own_list[0]
                del self.own_list[0]
                rospy.logwarn(f'{self.id}: {len(self.own_list)} left')
                rospy.wait_for_service(f'{self.id}/process_task')
                task_service = rospy.ServiceProxy(f'{self.id}/process_task',
                                                  ProcessTask)
                request = ProcessTaskRequest()
                request.task = self.next_target
                result = task_service(request)
                if result:
                    rospy.loginfo(
                        f'{self.id}: {self.next_target} has been cleared')
                if self.next_target.pose.position.x > -21.0:
                    goal = self.next_target
                    goal.header.stamp = rospy.Time.now()
                    action_goal = MoveBaseGoal()
                    action_goal.target_pose = goal
                    print(action_goal)
                    self.move_client.send_goal(action_goal,
                                               feedback_cb=self.feedback_callback)
                    path_found = self.move_client.wait_for_result(
                        rospy.Duration.from_sec(30.0))
                    # print(path_found)

                    if not path_found:
                        current_pose = rospy.wait_for_message(
                            f"{self.id}/amcl_pose",
                            PoseWithCovarianceStamped)
                        current_x = current_pose.pose.pose.position.x
                        current_y = current_pose.pose.pose.position.y
                        goal_x = self.next_target.pose.position.x
                        goal_y = self.next_target.pose.position.y
                        self.move_client.cancel_goal()
                        if not self.trigger_1:
                            if (current_x - goal_x) ** 2 + (
                                    current_y - goal_y) ** 2 > 2.25:
                                rospy.logwarn(f'{self.id}: TOO LONG TIME USED')
                                rospy.wait_for_service(f'{self.id}/fail_task')
                                fail_service = rospy.ServiceProxy(
                                    f'{self.id}/fail_task',
                                    FailTask)
                                fail_request = FailTaskRequest()
                                fail_request.task = self.next_target
                                result = fail_service(fail_request)
                    elif not self.cancelled:
                        rospy.loginfo(f'{self.id}: moving')
                        self.move_client.wait_for_result()
                    else:
                        self.cancelled = False
            elif not self.trigger_1 and self.received:
                rospy.loginfo(f'{self.id}: PRIMARY FINISHED')
                self.trigger_1 = True
            elif self.trigger_1 and not self.trigger_2:
                # rospy.loginfo(f'{self.id}: OUTER FINISHED')
                self.trigger_2 = True

            rate.sleep()

    def feedback_callback(self, feedback):
        if self.counter == 0:
            current_pose = rospy.wait_for_message(
                f"{self.id}/amcl_pose",
                PoseWithCovarianceStamped)
            self.current_pose = PoseStamped()
            self.current_pose.header = current_pose.header
            self.current_pose.pose = current_pose.pose.pose
        self.counter += 1
        if self.counter > 120:
            # print(feedback.base_position)
            self.counter = 0
            x_1 = feedback.base_position.pose.position.x
            y_1 = feedback.base_position.pose.position.y
            x_2 = self.current_pose.pose.position.x
            y_2 = self.current_pose.pose.position.y
            if (x_1 - x_2) ** 2 + (y_1 - y_2) ** 2 <= 0.1:
                rospy.logwarn(f'{self.id}: CAN NOT REACH GOAL')
                self.move_client.cancel_goal()
                rospy.wait_for_service(f'{self.id}/fail_task')
                fail_service = rospy.ServiceProxy(f'{self.id}/fail_task',
                                                  FailTask)
                fail_request = FailTaskRequest()
                fail_request.task = self.next_target
                result = fail_service(fail_request)
                self.cancelled = True

    def register_tasks(self, msg):
        rospy.loginfo(f'{self.id}: received new tasks from parent')
        self.received = True
        self.own_list = msg.tasks
        rospy.loginfo(f'{self.id}: CONTROLLER: {self.own_list}')

    def task_subscriber(self):
        tasks = rospy.Subscriber(f'{self.id}/own_tasks', TaskArray,
                                 self.register_tasks)


if __name__ == '__main__':
    node = Controller()
    node.perform_tasks()
    # rospy.spin()
