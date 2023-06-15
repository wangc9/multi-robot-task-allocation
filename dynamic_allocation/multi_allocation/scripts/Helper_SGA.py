#!/usr/bin/env python3

import copy
import pickle
import random

import numpy as np
from scipy.spatial import distance_matrix

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from multi_allocation.srv import AskWorkload
from multi_allocation.srv import AssignAuctioneer
from multi_allocation.srv import ClearAuctioneer, ClearAuctioneerResponse
from multi_allocation.srv import MainTaskAllocation, MainTaskAllocationRequest
from multi_allocation.srv import MultiTaskAllocation, MultiTaskAllocationRequest
from multi_allocation.srv import OuterSwap
from std_srvs.srv import Trigger, TriggerResponse


class SGA:
    def __init__(self, robot_num: int, task_num: int, tasks):
        self.robot_num = robot_num
        self.task_num = task_num
        self.task_file = tasks

        robots = []
        self.robot = None
        for i in range(robot_num):
            robot_pose = rospy.wait_for_message(f'/tb3_{i}/amcl_pose',
                                                PoseWithCovarianceStamped)
            robots.append([robot_pose.pose.pose.position.x,
                           robot_pose.pose.pose.position.y])
        self.robot = np.array(robots)
        tasks = []
        self.task = None
        for j in self.task_file:
            tasks.append([j.pose.position.x, j.pose.position.y])
        self.task = np.array(tasks)

        # Sequential Greedy Algorithm
        I = [i for i in range(len(self.robot))]
        J = [j for j in range(len(self.task))]

        bundle = [[] for i in range(len(self.robot))]
        path = [[] for i in range(len(self.robot))]
        Sp = [0 for i in range(len(self.robot))]

        L_t = len(self.task)
        N_min = min(len(self.task), len(self.robot) * L_t)
        alpha = 1
        penalty = 100

        eta = np.zeros(len(self.robot), dtype=np.int16)
        score = -distance_matrix(self.robot, self.task)

        for n in range(N_min):
            i_star, j_star = np.unravel_index(np.argmax(score, axis=None),
                                              score.shape)

            eta[i_star] += 1
            J.remove(j_star)

            # Construct Bundle
            bundle[i_star].append(j_star)

            # Construct Path
            if len(path[i_star]) > 0:
                c_list = []
                for k in range(len(path[i_star]) + 1):
                    p = copy.deepcopy(path[i_star])
                    p.insert(k, j_star)
                    c = 0
                    c += distance_matrix([self.robot[i_star]],
                                         [self.task[p[0]]]).squeeze()
                    if len(p) > 1:
                        for loc in range(len(p) - 1):
                            c += distance_matrix([self.task[p[loc]]],
                                                 [self.task[
                                                      p[loc + 1]]]).squeeze()
                    c = -(c - Sp[i_star])
                    c_list.append(c)

                idx = np.argmax(c_list)
                c_max = c_list[idx]

                path[i_star].insert(idx, j_star)
            else:
                path[i_star].append(j_star)

            Sp[i_star] = 0
            Sp[i_star] += distance_matrix([self.robot[i_star]],
                                          [self.task[
                                               path[i_star][0]]]).squeeze()
            if len(path[i_star]) > 1:
                for loc in range(len(path[i_star]) - 1):
                    Sp[i_star] += distance_matrix(
                        [self.task[path[i_star][loc]]], [
                            self.task[path[i_star][loc + 1]]]).squeeze()

            max_length = max(Sp)

            if eta[i_star] == L_t:
                I.remove(i_star)
                score[i_star, :] = -np.inf

            score[I, j_star] = -np.inf

            # Score Update
            for i in I:
                for j in J:
                    c_list = []
                    for k in range(len(path[i]) + 1):
                        p = copy.deepcopy(path[i])
                        p.insert(k, j)
                        c = 0
                        c += distance_matrix([self.robot[i]],
                                             [self.task[p[0]]]).squeeze()
                        if len(p) > 1:
                            for loc in range(len(p) - 1):
                                c += distance_matrix([self.task[p[loc]]], [
                                    self.task[p[loc + 1]]]).squeeze()

                        if c > max_length:
                            c = -(c - max_length) * penalty
                        else:
                            c = -(c - Sp[i])
                        c_list.append(c)

                    if c_list:
                        if i == i_star:
                            score[i, j] = (1 / alpha) * max(c_list)
                        else:
                            score[i, j] = max(c_list)
        print(path)
        for i in range(len(path)):
            if len(path[i]) > 0:
                rospy.wait_for_service(f'/tb3_{i}/multi_task_allocation')
                multi_allocation_service = rospy.ServiceProxy(
                    f'/tb3_{i}/multi_task_allocation', MultiTaskAllocation)
                if self.task_num == 1:
                    tasks = self.task_file
                else:
                    tasks = [self.task_file[j] for j in path[i]]
                request = MultiTaskAllocationRequest()
                request.tasks = tasks
                result = multi_allocation_service(request)
                if not result.result:
                    rospy.logerr(f'Allocation to tb3_{i} failed')
                else:
                    rospy.loginfo(f'Allocated to tb3_{i}')


class Helper:

    def __init__(self, manual: bool, file_path: str, known: bool):
        # self.manual = manual
        # self.file_path = file_path
        # self.known = known
        self.robots = []
        self.tasks = []
        self.file_tasks = []
        self.intervals = []
        self.auctioneer = None
        names = rosnode.get_node_names()
        for name in names:
            if "tb3" in name and "move_base" in name:
                self.robots.append(f'/{name.split("/")[1]}')
        print("Current robots in the system:")
        print(self.robots)
        # self.map_sub = rospy.Subscriber('/tb3_0/map', OccupancyGrid,
        #                                 self.register_distance)
        self.counter_1 = 0
        self.fail_tasks = []
        self.fail_task_counter = 0
        self.map = None
        self.map_np = None
        self.resolution = None
        self.width = None
        self.height = None
        self.distances_dict = None
        self.outer_swap()
        self.fail_counter()
        # self.count = 0
        # self.distances = dict()
        # self.register_distance()
        self.delete_auctioneer()
        if manual:
            self.listener()
        else:
            self.task_generator(file_path, known)

    def all_tasks(self):
        if len(self.tasks) == 0:
            rospy.loginfo("No task has been logged!")
        else:
            rospy.loginfo("All tasks registered in the system are as follow:")
            for task in self.tasks:
                rospy.loginfo(f'Task {task[0]}:')
                rospy.loginfo(task[2])

    def add_task_callback(self, data):
        num = len(self.tasks)
        rospy.loginfo(
            f'Task number {num + 1} received, [{data.pose.position.x},\
             {data.pose.position.y}, {data.pose.position.z}]')
        if self.auctioneer is None:
            self.ask_auctioneer()
        rospy.wait_for_service(f'{self.auctioneer}/main_task_allocation')
        rospy.loginfo(f'Allocation to {self.auctioneer} started:')
        main_task_allocation_service = rospy.ServiceProxy(
            f'{self.auctioneer}/main_task_allocation', MainTaskAllocation)
        main_allocation_request = MainTaskAllocationRequest()
        main_allocation_request.pose.header = data.header
        main_allocation_request.pose.pose = data.pose
        result = main_task_allocation_service(main_allocation_request)
        # if not result:
        #    rospy.loginfo(f'Task {data} failed to be allocated to\
        #     auctioneer {self.auctioneer}, trying to cancel...')
        #    try:
        #        rospy.wait_for_service(f'{self.auctioneer}/cancel_auction')
        #        cancel_auction_service = rospy.ServiceProxy(f'{self.auctioneer}\
        #        /cancel_auction', CancelAuction)
        #        cancel_result = cancel_auction_service()
        self.tasks.append([data.header.seq, data.pose, data])
        self.all_tasks()

    def task_generator(self, file_path: str, known: bool):
        file = open(file_path, 'rb')
        self.file_tasks = pickle.load(file)
        file.close()
        if not known:
            for task in self.file_tasks:
                interval = random.randrange(3, 5, 1)
                self.intervals.append(interval)
                SGA(len(self.robots), 1, [task])
                rospy.sleep(float(interval))
        else:
            SGA(len(self.robots), len(self.file_tasks), self.file_tasks)

    def listener(self):
        # while not rospy.is_shutdown():
        #     pose = rospy.wait_for_message("task_alloc", PoseStamped)
        #     self.add_task_callback(pose)
        rospy.Subscriber("task_alloc", PoseStamped, self.add_task_callback)

    def ask_auctioneer(self):
        workload = 99
        for robot in self.robots:
            rospy.wait_for_service(f'{robot}/ask_workload')
            ask_workload_service = rospy.ServiceProxy(f'{robot}/ask_workload',
                                                      AskWorkload)
            result = ask_workload_service()
            print(result)
            if result.workload < workload:
                self.auctioneer = result.id
                workload = result.workload
        print(f'The auctioneer is {self.auctioneer}')
        for robot in self.robots:
            rospy.wait_for_service(f'{robot}/assign_auctioneer')
            assign_auctioneer_service = rospy.ServiceProxy(
                f'{robot}/assign_auctioneer', AssignAuctioneer)
            assign_result = assign_auctioneer_service(self.auctioneer)
            if not assign_result:
                for i in range(5):
                    rospy.loginfo(f'Auctioneer broadcast to {robot} has failed,\
                     retrying...')
                    assign_auctioneer_service = rospy.ServiceProxy(
                        f'{robot}/assign_auctioneer', AssignAuctioneer)
                    assign_result = assign_auctioneer_service(self.auctioneer)
                    if assign_result:
                        break
                if not assign_result:
                    rospy.loginfo(f'Robot {robot} is not responding,\
                     suspending...')
                    self.robots.remove(robot)
            else:
                rospy.loginfo(
                    f'Set auctioneer for {robot} to {self.auctioneer}')

    def delete_auctioneer(self):
        delete_auc_service = rospy.Service(f'/helper/clear_auctioneer',
                                           ClearAuctioneer,
                                           self.delete_auctioneer_callback)

    def delete_auctioneer_callback(self, request):
        response = ClearAuctioneerResponse()
        self.auctioneer = None
        if self.auctioneer is None:
            rospy.loginfo(f'helper: auctioneer cleared')
            response.result = True
        else:
            response.result = False

        return response

    def fail_counter_callback(self, request):
        self.fail_task_counter += 1
        response = TriggerResponse()
        response.success = True
        response.message = 'Failed task registered'

        return response

    def fail_counter(self):
        fail_counter_service = rospy.Service('/helper/fail_task_counter',
                                             Trigger,
                                             self.fail_counter_callback)

    def outer_swap_callback(self, request):
        self.counter_1 += 1
        self.fail_tasks += request.tasks
        if self.counter_1 == len(self.robots):
            self.counter_1 = 0
            for task in self.fail_tasks:
                self.add_task_callback(task)

        return True

    def outer_swap(self):
        outer_swap_service = rospy.Service('/helper/outer_swap', OuterSwap,
                                           self.outer_swap_callback)


if __name__ == '__main__':
    rospy.init_node('helper_sga')
    manual = True if rospy.get_param('~manual', False) is True else False
    file_path = rospy.get_param('~file_path', None)
    known = True if rospy.get_param('~known', False) is True else False
    helper = Helper(manual, file_path, known)
    # helper.ask_auctioneer()
    # helper.listener()

    rospy.spin()
