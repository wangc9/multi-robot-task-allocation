#!/usr/bin/env python3

import math
import time

import numpy as np
from scipy.optimize import linear_sum_assignment

import rosnode
import rospy
from concorde import Problem, run_concorde
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from multi_allocation.msg import TaskArray
from multi_allocation.srv import AskWorkload, AskWorkloadResponse
from multi_allocation.srv import AssignAuctioneer, AssignAuctioneerResponse
from multi_allocation.srv import AuctionPropose, AuctionProposeResponse, \
    AuctionProposeRequest
from multi_allocation.srv import ClearAuctioneer, ClearAuctioneerResponse
from multi_allocation.srv import EstimateDistance, EstimateDistanceRequest
from multi_allocation.srv import FailTask, FailTaskResponse
from multi_allocation.srv import MainTaskAllocation, MainTaskAllocationResponse
from multi_allocation.srv import OuterSwap, OuterSwapRequest
from multi_allocation.srv import ProcessTask, ProcessTaskResponse
from multi_allocation.srv import SecondTaskAllocation, \
    SecondTaskAllocationResponse
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, Trigger


class Dealer:
    def __init__(self):
        self.init = True
        self.use_init = False
        self.moved = False
        self.current_goal = None
        self.world_frame = "map"
        rospy.init_node("dealer")
        self.own_list = []
        self.failed_list = []
        self.robot_list = []
        names = rosnode.get_node_names()
        for name in names:
            if "tb3" in name and "move_base" in name:
                self.robot_list.append(f'/{name.split("/")[1]}')
        self.id = f'/{rospy.get_name().split("/")[1]}'
        print(
            f'The name of this node is {self.id}, recorded robot{self.robot_list}')
        self.main_task_allocation_service = None
        self.fail_task_counter = 0
        self.distance_dict = dict()
        self.distances = dict()
        self.initial_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.temp_distance = 0.0
        self.covered_distance = 0.0
        self.map_sub = rospy.Subscriber(
            f'{self.id}/move_base/global_costmap/costmap',
            OccupancyGrid,
            self.map_callback)
        self.outer_swap_enabled = False
        self.costmap_np = None
        self.costmap = None
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None
        self.pub = rospy.Publisher(f'{self.id}/own_tasks', TaskArray,
                                   queue_size=10)
        self.d_lite = None
        # self.register_distance()
        self.receive_task()
        self.delete_auctioneer()
        self.in_bidding = False
        self.is_auctioneer = False
        self.auctioneer = None
        self.task_for_auction = []
        self.evaluate_to_be_auctioneer()
        self.auctioneer_information()
        self.auction_propose()
        self.fail_task()
        self.process_task()
        # if self.is_auctioneer:
        # self.add_auction_task()
        # self.auction()

    def workload_callback(self, request):
        rospy.loginfo(f'Robot {self.id} received auctioneer request')
        current_load = len(self.own_list)
        response = AskWorkloadResponse()
        response.id = self.id
        response.workload = current_load

        return response

    def assign_auctioneer_callback(self, request):
        self.auctioneer = request.id
        if self.auctioneer == self.id:
            self.is_auctioneer = True
        response = AssignAuctioneerResponse()
        response.status = True
        if self.is_auctioneer:
            rospy.loginfo(f'{self.id}: I am auctioneer.')
            self.add_auction_task()
        else:
            rospy.loginfo(f'{self.id}: Auctioneer is {self.auctioneer}')

        return response

    def add_auction_task_callback(self, request):
        response = MainTaskAllocationResponse()
        if not self.is_auctioneer:
            response.status = False
            rospy.loginfo(f'{self.id}: I am NOT auctioneer!')
        else:
            self.task_for_auction.append(request.pose)
            response.status = True
            rospy.loginfo(f'Task\n {request} \nreceived')
            self.auction()

        return response

    def evaluate_to_be_auctioneer(self):
        workload_service = rospy.Service(f'{self.id}/ask_workload', AskWorkload,
                                         self.workload_callback)

    def auctioneer_information(self):
        assign_auctioneer_service = rospy.Service(
            f'{self.id}/assign_auctioneer',
            AssignAuctioneer,
            self.assign_auctioneer_callback)

    def add_auction_task(self):
        self.main_task_allocation_service = rospy.Service(
            f'{self.id}/main_task_allocation', MainTaskAllocation,
            self.add_auction_task_callback)

    def auction(self):
        while len(self.task_for_auction) != 0:
            rospy.loginfo(f'{self.id} is acting as auctioneer:')
            if len(self.task_for_auction) == 1:
                task_for_allocation = self.task_for_auction
            else:
                task_for_allocation = self.task_for_auction[
                                      0:min(len(self.robot_list),
                                            len(self.task_for_auction))]
            rospy.loginfo(task_for_allocation)
            bids_temp = []
            allocation = []
            no_auction = False
            for robot in self.robot_list:
                rospy.loginfo(f'Auctioneer {self.id} is waiting for {robot}')
                rospy.wait_for_service(f'{robot}/auction_propose')
                auction_propose_service = rospy.ServiceProxy(
                    f'{robot}/auction_propose', AuctionPropose)
                propose_request = AuctionProposeRequest()
                for unalloc_task in task_for_allocation:
                    if unalloc_task.pose.position.x <= -21.0:
                        rospy.wait_for_service(
                            f'{robot}/task_allocation')
                        alloc_service = rospy.ServiceProxy(
                            f'{robot}/task_allocation',
                            SecondTaskAllocation)
                        alloc_result = alloc_service(unalloc_task)
                        rospy.loginfo(f'{robot}: FINAL task added')
                        if unalloc_task not in allocation:
                            allocation.append(unalloc_task)
                        no_auction = True
                    else:
                        propose_request.tasks.append(unalloc_task)
                # print(propose_request)
                if not no_auction:
                    assign_result = auction_propose_service(propose_request)
                    if assign_result is not None:
                        rospy.loginfo(f'{self.id}: {robot} has proposed')
                    bids_temp.append(assign_result.proposes)
            if not no_auction:
                bids_numpy = np.array(bids_temp)
                # print(self.robot_list)
                rospy.loginfo(bids_numpy)
                best_robot = linear_sum_assignment(-bids_numpy.T)[1]
                print(best_robot)
                for i in range(len(self.robot_list)):
                    task_num = np.where(best_robot == i)[0]
                    if len(task_num) > 0:
                        # if self.id != self.robot_list[i]:
                        #     print(f'{self.id}: WRONG!!! SHOULD BE {self.robot_list[i]}')
                        task = task_for_allocation[task_num[0]]
                        rospy.wait_for_service(
                            f'{self.robot_list[i]}/task_allocation')
                        alloc_service = rospy.ServiceProxy(
                            f'{self.robot_list[i]}/task_allocation',
                            SecondTaskAllocation)
                        alloc_result = alloc_service(task)
                        if alloc_result.result:
                            rospy.loginfo(f'Auctioneer {self.id}: task {task} allocated\
                             to robot {self.robot_list[i]}')
                            allocation.append(task)
                        else:
                            print(f'!!!! Failed to allocate task {task} to robot \
                            {self.robot_list[i]}')
                    else:
                        rospy.loginfo(
                            f'{self.robot_list[i]} skipped because less compatible')
            for item in allocation:
                self.task_for_auction.remove(item)
            allocation = []
            task_for_allocation = []
        rospy.loginfo(f'Auctioneer {self.id}: Allocation finished, returning \
        auctioneer status')
        self.clear_auctioneer()

    def clear_auctioneer(self):
        for robot in self.robot_list:
            rospy.wait_for_service(f'{robot}/clear_auctioneer')
            clear_service = rospy.ServiceProxy(f'{robot}/clear_auctioneer',
                                               ClearAuctioneer)
            clear_result = clear_service()
            if clear_result:
                rospy.loginfo(f'{self.id}: {robot} cleared of auctioneer')
        self.main_task_allocation_service.shutdown('clearing auctioneer ...')
        self.main_task_allocation_service = None
        rospy.wait_for_service(f'/helper/clear_auctioneer')
        clear_service = rospy.ServiceProxy(f'/helper/clear_auctioneer',
                                           ClearAuctioneer)
        rospy.loginfo("clearing helper:")
        clear_result = clear_service()
        if clear_result:
            rospy.loginfo(f'helper cleared of auctioneer')

    def delete_auctioneer(self):
        delete_auc_service = rospy.Service(f'{self.id}/clear_auctioneer',
                                           ClearAuctioneer,
                                           self.delete_auctioneer_callback)

    def delete_auctioneer_callback(self, request):
        response = ClearAuctioneerResponse()
        self.auctioneer = None
        self.in_bidding = False
        self.is_auctioneer = False
        if self.auctioneer is None:
            rospy.loginfo(f'{self.id}: auctioneer cleared')
            response.result = True
        else:
            response.result = False

        return response

    def receive_task(self):
        receive_service = rospy.Service(f'{self.id}/task_allocation',
                                        SecondTaskAllocation,
                                        self.receive_task_callback)

    def process_task_callback(self, req):
        task = req.task
        task_index = task.header.seq
        self.own_list = [tasks for tasks in self.own_list if
                         tasks.header.seq != task_index]
        response = ProcessTaskResponse()
        response.status = True
        self.current_goal = task
        self.moved = True
        rospy.loginfo(f"{self.id}: I am going to move, changed current goal")

        return response

    def process_task(self):
        process_task_service = rospy.Service(f'{self.id}/process_task',
                                             ProcessTask,
                                             self.process_task_callback)

    def fail_task_callback(self, req):
        if self.outer_swap_enabled:
            self.fail_task_counter += 1
            rospy.wait_for_service('/helper/fail_task_counter')
            counter_service = rospy.ServiceProxy('helper/fail_task_counter',
                                                 Trigger)
            result = counter_service()
            if result.success:
                rospy.loginfo(f'{self.id}: added fail number')
        else:
            task = req.task
            self.failed_list.append(task)
            current_pose = rospy.wait_for_message(f"{self.id}/amcl_pose",
                                                  PoseWithCovarianceStamped)
            self.current_pose.header = current_pose.header
            self.current_pose.pose = current_pose.pose.pose
            rospy.loginfo(f'{self.id}: task {task} FAILED and pushed to stack')
        response = FailTaskResponse()
        response.status = True

        return response

    def fail_task(self):
        fail_task_service = rospy.Service(f'{self.id}/fail_task',
                                          FailTask,
                                          self.fail_task_callback)

    def receive_task_callback(self, request):
        response = SecondTaskAllocationResponse()
        try:
            if request.task.pose.position.x <= -21.0:
                self.outer_swap_enabled = True
                rospy.loginfo(f'{self.id}: FINAL task received')
                rospy.wait_for_message('/helper/outer_swap')
                outer_swap_service = rospy.ServiceProxy('/helper/outer_swap',
                                                        OuterSwap)
                rospy.loginfo(f'{self.id}: OUTERSWAP')
                outer_request = OuterSwapRequest()
                outer_request.tasks = self.failed_list
                result = outer_swap_service(outer_request)
            else:
                self.own_list.append(request.task)
        except AttributeError or AssertionError:
            rospy.loginfo(f'!!!!{self.id}: Fail to receive task {request.task}')
            response.result = False
        else:
            response.result = True
        finally:
            self.use_init = False
            if not self.outer_swap_enabled:
                rospy.loginfo(
                    f'{self.id}: original distance: {self.covered_distance}')
                # rospy.loginfo(
                #     f'{self.id}: current initial pose: {self.initial_pose}')
                self.covered_distance += self.temp_distance
                rospy.loginfo(
                    f'{self.id}: distance changed to {self.covered_distance}')
                # rospy.loginfo(
                #     f'{self.id}: current initial pose: {self.initial_pose}')
                self.current_pose = request.task
                if len(self.own_list) >= 3:
                    rospy.loginfo(f'{self.id}: task swapping')
                    # rospy.loginfo(f'{self.id}: original order: {self.own_list}')
                    self.inner_swap_task()
                    # rospy.loginfo(f'{self.id}: new order: {self.own_list}')
                self.pub.publish(self.own_list)
            return response

    def auction_propose(self):
        auction_propose_service = rospy.Service(f'{self.id}/auction_propose',
                                                AuctionPropose,
                                                self.auction_propose_callback)

    def auction_propose_callback(self, request):
        result = []
        self.get_pose()
        rospy.logdebug(
            f'{self.id}: received task {request.tasks}, will plan from {self.current_pose}')
        # start_pose = PoseStamped()
        # start_pose.pose = self.current_pose
        # start_pose.header.frame_id = "map"
        # start_pose.header.stamp = rospy.Time.now()
        # length = Float32()
        respond = AuctionProposeResponse()
        for task in request.tasks:
            rospy.wait_for_service(f'{self.id}/distance_estimator')
            get_dist = rospy.ServiceProxy(
                f'{self.id}/distance_estimator', EstimateDistance)
            request = EstimateDistanceRequest()
            request.start = self.current_pose
            request.end = task
            result = get_dist(request)
            t_dist = result.distance
            # rospy.wait_for_service(f'{self.id}/move_base/make_plan')
            # get_plan = rospy.ServiceProxy(f'{self.id}/move_base/make_plan',
            #                               GetPlan)
            # req = GetPlanRequest()
            # req.start = self.current_pose
            # req.goal = task
            # req.tolerance = 0.5
            # temp_plan = get_plan(req)
            # length.data = float(-self.calculate_path_length(temp_plan.plan))
            # print(length)
            # print(type(length))
            rospy.loginfo(f'{self.id}: current pose: {self.current_pose}')
            rospy.loginfo(
                f'{self.id}: original distance for proposal: {self.covered_distance}')
            # self.temp_distance = -self.calculate_path_length(temp_plan.plan)
            self.temp_distance = -t_dist
            # register initial pose as -1
            if self.use_init:
                self.add_distance_to_dict(-1, task.header.seq,
                                          -self.temp_distance)
            else:
                a = self.current_pose.header.seq
                self.add_distance_to_dict(a, task.header.seq,
                                          -self.temp_distance)
            respond.proposes.append(
                float(self.covered_distance + self.temp_distance))
        # print(respond)
        # print(type(respond))

        return respond

    def add_distance_to_dict(self, i: int, j: int, dist: float):
        """
        Add the calculated distance to `self.distance_dict`
        :param i: int, start index
        :param j: int, start index
        :param dist: float, calculated distance
        :return: None
        """
        # Add distance for dict[i][j]
        if i not in self.distance_dict:
            self.distance_dict[i] = dict()
            self.distance_dict[i][j] = dist
        else:
            self.distance_dict[i][j] = dist

        # Add distance for dict[j][i]
        if j not in self.distance_dict:
            self.distance_dict[j] = dict()
            self.distance_dict[j][i] = dist
        else:
            self.distance_dict[j][i] = dist
        # rospy.loginfo(f'{self.id}: distance dict: {self.distance_dict}')

    def inner_swap_task(self):
        """
        Task swap inside a single robot
        :return: In-place change of `self.own_list`
        """
        tic = time.perf_counter()
        # rospy.loginfo(f'{self.id}: initial pose {self.initial_pose}')
        temp_list = self.own_list

        # Push start point (initial pose or current goal) to end
        if not self.moved:
            temp_list.append(self.initial_pose)
        else:
            temp_list.append(self.current_goal)
        temp = []
        temp_all = []
        tour_1 = []
        tour_2 = []
        a = len(temp_list)

        # Construct a symmetric matrix
        for i in range(a):
            for j in range(a):
                # diagonal
                if i == j:
                    temp.append(0.0)
                # bottom-left corner mimics the up-right corner
                elif i > j:
                    temp.append(temp_all[j][i])
                # up-right corner
                else:
                    # rospy.loginfo(
                    #     f'{self.id}: calculating from {self.own_list[i]} to {self.own_list[j]}')
                    rospy.wait_for_service(f'{self.id}/distance_estimator')
                    get_dist = rospy.ServiceProxy(
                        f'{self.id}/distance_estimator', EstimateDistance)
                    request = EstimateDistanceRequest()
                    # rospy.wait_for_service(f'{self.id}/move_base/make_plan')
                    # get_plan = rospy.ServiceProxy(
                    #     f'{self.id}/move_base/make_plan',
                    #     GetPlan)
                    # req = GetPlanRequest()
                    # req.start = self.own_list[i]
                    # req.goal = self.own_list[j]
                    # req.tolerance = 0.5
                    # change index of task i and j to -1 if is initial pose
                    if i != a - 1:
                        n_1 = self.own_list[i].header.seq
                    else:
                        n_1 = -1
                    if j != a - 1:
                        n_2 = self.own_list[j].header.seq
                    else:
                        n_2 = -1
                    if n_2 in self.distance_dict[n_1]:
                        temp.append(self.distance_dict[n_1][n_2])
                    else:
                        # temp_plan = get_plan(req)
                        # t_dist = self.calculate_path_length(temp_plan.plan)
                        # Distance hasn't been registered in dict, calculate new distances
                        request.start = temp_list[i]
                        request.end = temp_list[j]
                        result = get_dist(request)
                        t_dist = result.distance
                        temp.append(t_dist)
                        self.add_distance_to_dict(n_1, n_2, t_dist)
            temp_all.append(temp)
            temp = []
        distance_array = np.array(temp_all)
        rospy.loginfo(f'{self.id}: distance array: {distance_array}')

        # Run concorde solver
        problem = Problem.from_matrix(distance_array)
        solution = run_concorde(problem)
        tour = solution.tour

        # Re-arrange tour to start after initial pose
        if tour.index(a - 1) != a - 1:
            tour_1 = tour[tour.index(a - 1) + 1:] + tour[:tour.index(a - 1)]
        else:
            tour_1 = tour[:-1]
        # Reverse tour
        tour_2 = tour_1[::-1]
        print("*********************")
        print("tour", tour)
        print("tour1", tour_1)
        print("tour2", tour_2)
        print("*********************")

        # Real tour
        own_list_1 = [self.own_list[i] for i in tour_1]
        own_list_2 = [self.own_list[i] for i in tour_2]
        covered_distance_1 = 0.0
        covered_distance_2 = 0.0
        self.covered_distance = 0.0
        # rospy.wait_for_service(f'{self.id}/move_base/make_plan')
        # get_plan = rospy.ServiceProxy(f'{self.id}/move_base/make_plan',
        #                               GetPlan)
        # req = GetPlanRequest()
        # req.start = self.initial_pose
        # req.goal = own_list_1[0]
        t_1 = own_list_1[0].header.seq
        t_2 = self.distance_dict[-1]  # All distance from start point
        if t_1 in t_2:
            covered_distance_1 = -t_2[t_1]
        else:
            rospy.wait_for_service(f'{self.id}/distance_estimator')
            get_dist = rospy.ServiceProxy(
                f'{self.id}/distance_estimator', EstimateDistance)
            request = EstimateDistanceRequest()
            request.start = self.current_goal if self.moved else self.initial_pose
            request.end = own_list_1[0]
            result = get_dist(request)
            dist = result.distance
            # dist = self.d_lite.generate_path()
            # temp_path, dist = self.plan_path(self.initial_pose, own_list_1[0])
            # temp_plan = get_plan(req)
            # dist = self.calculate_path_length(temp_plan.plan)
            covered_distance_1 = -dist
            self.add_distance_to_dict(-1, t_1, dist)
            # print("initial distance 1", covered_distance_1)

        # req.goal = own_list_2[0]
        t_1 = own_list_2[0].header.seq
        t_2 = self.distance_dict[-1]
        if t_1 in t_2:
            covered_distance_2 = -t_2[t_1]
        else:
            rospy.wait_for_service(f'{self.id}/distance_estimator')
            get_dist = rospy.ServiceProxy(
                f'{self.id}/distance_estimator', EstimateDistance)
            request = EstimateDistanceRequest()
            request.start = self.current_goal if self.moved else self.initial_pose
            request.end = own_list_2[0]
            result = get_dist(request)
            dist = result.distance
            # temp_path, dist = self.plan_path(self.initial_pose, own_list_2[0])
            # temp_plan = get_plan(req)
            # dist = self.calculate_path_length(temp_plan.plan)
            covered_distance_2 = -dist
            self.add_distance_to_dict(-1, t_1, dist)
        # print("initial distance 2", covered_distance_1)

        rospy.loginfo(
            f'{self.id}: dist1: {covered_distance_1}, dist2: {covered_distance_2}')
        if covered_distance_1 <= covered_distance_2:
            self.own_list = own_list_2
            self.covered_distance = covered_distance_2
        else:
            self.own_list = own_list_1
            self.covered_distance = covered_distance_1
        for i in range(len(self.own_list)):
            task_seq = self.own_list[i].header.seq
            if i == 0:
                self.covered_distance -= self.distance_dict[-1][task_seq]
            else:
                prev_task_seq = self.own_list[i - 1].header.seq
                self.covered_distance -= self.distance_dict[prev_task_seq][
                    task_seq]
        rospy.loginfo(
            f'{self.id}: new covered distance: {self.covered_distance}')

        toc = time.perf_counter()
        print(f"**************** Used {toc - tic} seconds! *******************")

        # dist = 10000
        # combs = None
        # temp_dists = []
        # temp_dist = 0.0
        # temp = self.initial_pose
        # for comb in itertools.permutations(self.own_list):
        #     start_pose = PoseStamped()
        #     start_pose.pose = temp.pose
        #     start_pose.header = temp.header
        #     start_pose.header.stamp = rospy.Time.now()
        #     for i in comb:
        #         rospy.wait_for_service(f'{self.id}/move_base/make_plan')
        #         get_plan = rospy.ServiceProxy(f'{self.id}/move_base/make_plan',
        #                                       GetPlan)
        #         req = GetPlanRequest()
        #         req.start = start_pose
        #         req.goal = i
        #         req.tolerance = 0.5
        #         temp_plan = get_plan(req)
        #         temp_dist += self.calculate_path_length(temp_plan.plan)
        #         start_pose = i
        #     temp_dists.append(temp_dist)
        #     if temp_dist < dist:
        #         dist = temp_dist
        #         combs = comb
        #
        # self.own_list = combs
        # rospy.loginfo(f'{self.id}: current plan: {self.own_list}')

    def calculate_path_length(self, path):
        first_time = True
        prev_x = 0.0
        prev_y = 0.0
        total_distance = 0.0
        # print(path)
        for current_point in path.poses:
            x = current_point.pose.position.x
            y = current_point.pose.position.y
            if not first_time:
                total_distance += math.hypot(prev_x - x, prev_y - y)
            else:
                first_time = False
            prev_x = x
            prev_y = y

        print(total_distance)
        print(type(total_distance))

        return total_distance

    def get_pose(self):
        if len(self.own_list) > 0:
            self.current_pose.pose = self.own_list[-1].pose
            self.current_pose.header.seq = self.own_list[-1].header.seq
            self.current_pose.header.stamp = rospy.Time.now()
            # rospy.loginfo(
            #     f'{self.id}: Start planning from: {self.initial_pose}')
            # rospy.loginfo(f'{self.id}: current pose: {self.current_pose}')
        elif len(self.own_list) == 0 and self.moved:
            self.current_pose.header.stamp = rospy.Time.now()
            rospy.loginfo(
                f'{self.id}: Starting planning from: {self.current_pose}')
        elif self.init:
            initial_pose = rospy.wait_for_message(f"{self.id}/amcl_pose",
                                                  PoseWithCovarianceStamped)
            self.current_pose.pose = initial_pose.pose.pose
            self.current_pose.header = initial_pose.header
            self.initial_pose.pose = initial_pose.pose.pose
            self.initial_pose.header = initial_pose.header
            self.init = False
            self.use_init = True
            rospy.loginfo(
                f'{self.id}: initial pose: {self.initial_pose}')
            rospy.wait_for_service(f'{self.id}/move_base/clear_costmaps')
            clear_map = rospy.ServiceProxy(
                f'{self.id}/move_base/clear_costmaps', Empty)
            result = clear_map()
            # pose_sub = None
            # pose_sub = rospy.Subscriber(f"{self.id}/amcl_pose",
            #                             PoseWithCovarianceStamped,
            #                             self.get_pose_callback, pose_sub)
        else:
            self.current_pose.pose = self.initial_pose.pose
            self.current_pose.header.stamp = rospy.Time.now()
            rospy.loginfo(
                f'{self.id}: current initial pose: {self.initial_pose}')

    def map_callback(self, data):
        # Store the map and its properties
        self.costmap_np = np.array(data.data).reshape(
            (data.info.height, data.info.width))
        self.costmap = data
        self.origin = data.info.origin
        self.resolution = data.info.resolution
        self.width = data.info.width
        self.height = data.info.height
        print(
            f'resolution: {self.resolution}\nwidth: {self.width}\nheight: {self.height}\norigin: {self.origin}')

    #
    # def map_to_world(self, point):
    #     x = point[0] * self.resolution + self.costmap.info.origin.position.x
    #     y = point[1] * self.resolution + self.costmap.info.origin.position.y
    #     return x, y
    #
    # def get_neighbors(self, cell):
    #     neighbors = []
    #     for dx in range(-1, 2):
    #         for dy in range(-1, 2):
    #             # Ignore the cell itself
    #             if dx == 0 and dy == 0:
    #                 continue
    #             x = cell[0] + dx
    #             y = cell[1] + dy
    #             # Check if the neighbor is within the bounds of the costmap
    #             if x >= 0 and x < self.costmap.info.width and y >= 0 and y < self.costmap.info.height:
    #                 # Check if the neighbor is not an obstacle
    #                 cost = self.costmap.data[y * self.costmap.info.width + x]
    #                 if cost != 100:
    #                     neighbors.append((x, y))
    #     return neighbors
    #
    # def cost(self, cell1, cell2):
    #     # Compute the Euclidean distance between the cells
    #     distance = np.sqrt(
    #         (cell1[0] - cell2[0]) ** 2 + (cell1[1] - cell2[1]) ** 2)
    #     # Compute the cost of moving from cell1 to cell2 as the maximum of the cell costs
    #     cost = max(
    #         self.costmap.data[cell1[1] * self.costmap.info.width + cell1[0]],
    #         self.costmap.data[cell2[1] * self.costmap.info.width + cell2[0]])
    #     return cost * distance
    #
    # def heuristic(self, cell, goal):
    #     # Compute the Euclidean distance between the cell and the goal
    #     distance = np.sqrt((cell[0] - goal[0]) ** 2 + (cell[1] - goal[1]) ** 2)
    #     # Compute the heuristic as the maximum of the cell cost and 1
    #     heuristic = max(
    #         self.costmap.data[cell[1] * self.costmap.info.width + cell[0]], 1)
    #     return heuristic * distance
    #
    # def plan_path(self, start_pose, goal_pose):
    #     # Convert the start and goal poses to map coordinates
    #     start_cell = self.world_to_map(
    #         (start_pose.pose.position.x, start_pose.pose.position.y))
    #     goal_cell = self.world_to_map(
    #         (goal_pose.pose.position.x, goal_pose.pose.position.y))
    #     print("coordinate conversion finished")
    #
    #     # Initialize the open and closed sets
    #     open_set = [(0, start_cell)]
    #     came_from = {}
    #     g_score = {start_cell: 0}
    #     f_score = {start_cell: self.heuristic(start_cell, goal_cell)}
    #
    #     while open_set:
    #         # Pop the cell with the lowest f-score
    #         current_f, current_cell = heapq.heappop(open_set)
    #         print(current_cell)
    #
    #         # Check if we have reached the goal
    #         if current_cell == goal_cell:
    #             print("goal reached")
    #             # Reconstruct the path
    #             path = [self.map_to_world(goal_cell)]
    #             distance = 0.0
    #             while current_cell in came_from:
    #                 distance += np.sqrt((current_cell[0] - came_from[current_cell][0]) ** 2 + (current_cell[1] - came_from[current_cell][1]) ** 2)
    #                 path.append(self.map_to_world(current_cell))
    #                 current_cell = came_from[current_cell]
    #             path.append(self.map_to_world(start_cell))
    #             path.reverse()
    #             return path, distance
    #
    #         # Expand the current cell
    #         for neighbor in self.get_neighbors(current_cell):
    #             print(f'neighbor: {neighbor}')
    #             # Compute the tentative g-score for the neighbor
    #             tentative_g = g_score[current_cell] + self.cost(current_cell,
    #                                                             neighbor)
    #
    #             # Check if the neighbor has already been evaluated or if the tentative g-score is better
    #             if neighbor in g_score and tentative_g >= g_score[neighbor]:
    #                 continue
    #
    #             # Add the neighbor to the open set if it is not already there
    #             if neighbor not in [x[1] for x in open_set]:
    #                 heapq.heappush(open_set, (f_score.get(neighbor, float('inf')), neighbor))
    #
    #             # Update the came_from, g_score, and f_score dictionaries
    #             came_from[neighbor] = current_cell
    #             g_score[neighbor] = tentative_g
    #             f_score[neighbor] = tentative_g + self.heuristic(neighbor,
    #                                                              goal_cell)
    #
    #     # If we reach this point, there is no path from the start to the goal
    #     return None, None

    # def register_distance(self):
    #     ogm = OccupancyGridManager(f"{self.id}/move_base/global_costmap/costmap",
    #                                subscribe_to_updates=False)
    #     available_grid = []
    #     x = range(ogm.width)
    #     y = range(ogm.height)
    #     for i in x:
    #         for j in y:
    #             if ogm.get_cost_from_costmap_x_y(i, j) != 100:
    #                 available_grid.append((i, j))
    #     u = range(len(available_grid))
    #     seq = 0
    #     seq_2 = 0
    #     for a in u:
    #         for b in range(a+1, len(available_grid)):
    #             a_x, a_y = ogm.get_world_x_y(available_grid[a][0],
    #                                          available_grid[a][1])
    #             b_x, b_y = ogm.get_world_x_y(available_grid[b][0],
    #                                          available_grid[b][1])
    #             rospy.wait_for_service(f'{self.id}/move_base/make_plan')
    #             get_plan = rospy.ServiceProxy(f'{self.id}/move_base/make_plan',
    #                                           GetPlan)
    #             req = GetPlanRequest()
    #             start = PoseStamped()
    #             end = PoseStamped()
    #             start.header.seq = seq
    #             start.header.frame_id = "map"
    #             start.header.stamp = rospy.Time.now()
    #             start.pose.position.x = a_x
    #             start.pose.position.y = a_y
    #             start.pose.position.z = 0.0
    #             start.pose.orientation.x = 0.0
    #             start.pose.orientation.y = 0.0
    #             start.pose.orientation.z = 0.0
    #             start.pose.orientation.w = 0.0
    #
    #             end.header.seq = seq_2
    #             end.header.frame_id = "map"
    #             end.header.stamp = rospy.Time.now()
    #             end.pose.position.x = b_x
    #             end.pose.position.y = b_y
    #             end.pose.position.z = 0.0
    #             end.pose.orientation.x = 0.0
    #             end.pose.orientation.y = 0.0
    #             end.pose.orientation.z = 0.0
    #             end.pose.orientation.w = 0.0
    #
    #             req.start = start
    #             req.goal = end
    #             req.tolerance = 0.5
    #
    #             temp_plan = get_plan(req)
    #             dist = self.calculate_path_length(temp_plan.plan)
    #             self.distances[(a_x, a_y)] = self.distances.get((a_x, a_y), dict())
    #             self.distances[(a_x, a_y)][(b_x, b_y)] = dist
    #             seq_2 += 1
    #         seq_2 = 0
    #         seq += 1
    #     print(self.distances)

    # def get_pose_callback(self, msg, subscriber):
    #     self.current_pose.pose = msg.pose.pose
    #     self.current_pose.header = msg.header
    #     self.initial_pose = self.current_pose
    #     self.init = False
    #     subscriber.unregister()
    #     rospy.loginfo(f'{self.id}: initial pose: {self.initial_pose}')


# class Node:
#     def __init__(self, x, y, g=inf, rhs=inf):
#         self.x = x
#         self.y = y
#         self.g = g
#         self.rhs = rhs
#         self.key = [inf, inf]
#         self.parent = None
#
#     def __lt__(self, other):
#         return self.key < other.key
#
#     def __eq__(self, other):
#         return self.x == other.x and self.y == other.y
#
#
# class DStarLitePlanner:
#     def __init__(self, costmap, origin, resolution, start, goal):
#         # self.map_sub = rospy.Subscriber(f'/move_base/global_costmap/costmap',
#         #                                 OccupancyGrid, self.map_callback)
#         # self.path_pub = rospy.Publisher('/d_star_lite_path', Path,
#         #                                 queue_size=10)
#         # self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped,
#         #                                  self.goal_callback)
#
#         self.map = costmap
#         self.origin = origin
#         self.resolution = resolution
#         self.start = self.world_to_map(
#             (start.pose.position.x, start.pose.position.y))
#         self.goal = self.world_to_map(
#             (goal.pose.position.x, goal.pose.position.y))
#         self.path = Path()
#         self.header = Header()
#         self.header.frame_id = 'map'
#         self.nodes = dict()
#
#     # def map_callback(self, msg):
#     #     self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
#
#     def world_to_map(self, point):
#         x = int(
#             (point[0] - self.origin.position.x) / self.resolution)
#         y = int(
#             (point[1] - self.origin.position.y) / self.resolution)
#         return (x, y)
#
#     # def goal_callback(self, msg):
#     #     self.goal = (int(msg.pose.position.x / msg.info.resolution),
#     #                  int(msg.pose.position.y / msg.info.resolution))
#     #     return self.generate_path()
#
#     def generate_path(self):
#         if self.map is None or self.goal is None:
#             return
#
#         start = self.start
#         if start is None:
#             start = self.get_nearest_free_node(self.goal)
#
#         path = self.d_star_lite(start, self.goal)
#         if path is None:
#             return None, inf  # Return None and infinity distance if no path is found
#         distance = 0  # Initialize distance to zero
#         for num in range(1, len(path)):
#             node = path[num - 1]
#             next_node = path[num]
#             distance += self.cost(node,
#                                   next_node)  # Add cost of edge to distance
#         distance *= self.resolution
#         print(f'path distance: {distance}')
#         # node = start
#         # while node != self.goal:
#         #     next_node = node.parent
#         #     distance += self.cost(node,
#         #                           next_node)  # Add cost of edge to distance
#         #     path.append(node)
#         #     node = next_node
#         # path.append(self.goal)
#         # distance += self.cost(path[-2],
#         #                       self.goal)  # Add cost of last edge to distance
#         # path.reverse()
#         self.path = path
#         # poses = []
#         # for node in path:
#         #     pose = PoseStamped()
#         #     pose.header = self.header
#         #     pose.pose.position.x = node.x * msg.info.resolution
#         #     pose.pose.position.y = node.y * msg.info.resolution
#         #     pose.pose.orientation.w = 1
#         #     poses.append(pose)
#         # self.path.poses = poses
#         return distance  # Return path and distance
#
#     def get_nearest_free_node(self, node):
#         queue = [(0, node)]
#         visited = set()
#
#         while queue:
#             cost, current = heappop(queue)
#             if self.map[current] == 0:
#                 return current
#
#             visited.add(current)
#
#             neighbors = self.get_neighbors(current)
#             for neighbor in neighbors:
#                 if neighbor in visited:
#                     continue
#                 if self.map[neighbor] == -1:
#                     continue
#
#                 distance = sqrt(
#                     (neighbor[0] - node[0]) ** 2 + (neighbor[1] - node[1]) ** 2)
#                 heappush(queue, (cost + distance, neighbor))
#
#     def get_neighbors(self, node):
#         neighbors = []
#         for dx in [-1, 0, 1]:
#             for dy in [-1, 0, 1]:
#                 if dx == 0 and dy == 0:
#                     continue
#                 x = node.x + dx
#                 y = node.y + dy
#                 if x < 0 or x >= self.map.shape[0] or y < 0 or y >= \
#                         self.map.shape[1]:
#                     continue
#                 if self.map[x, y] == -1:
#                     continue
#                 neighbors.append(self.get_node(x, y))
#         return neighbors
#
#     def heuristic(self, node1, node2):
#         return sqrt(
#             (node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
#
#     def cost(self, node1, node2):
#         dx = abs(node1.x - node2.x)
#         dy = abs(node1.y - node2.y)
#         if dx + dy == 1:
#             return 1
#         else:
#             return sqrt(2)
#
#     def get_node(self, x, y):
#         if (x, y) not in self.nodes:
#             self.nodes[(x, y)] = Node(x, y)
#         return self.nodes[(x, y)]
#
#     def d_star_lite(self, start, goal):
#         open_list = []
#         k_old = 0
#
#         def calculate_key(node_1):
#             node_1.key[0] = min(node_1.g, node_1.rhs) + self.heuristic(node_1,
#                                                                        goal_node) + k_old
#             node_1.key[1] = min(node_1.g, node_1.rhs)
#
#         def update_node(node_2):
#             if node_2 != goal_node:
#                 node_2.rhs = inf
#                 for neighbors in self.get_neighbors(node_2):
#                     if neighbors.g + self.cost(node_2, neighbors) < node_2.rhs:
#                         node_2.rhs = neighbors.g + self.cost(node_2, neighbors)
#                         node_2.parent = neighbors
#
#             if node_2 in open_list:
#                 open_list.remove(node_2)
#
#             if node_2.g != node_2.rhs:
#                 calculate_key(node_2)
#                 heappush(open_list, node_2)
#
#         start_node = self.get_node(start[0], start[1])
#         goal_node = self.get_node(goal[0], goal[1])
#         goal_node.rhs = 0
#         open_list.append(goal_node)
#         calculate_key(start_node)
#         while open_list and (open_list[
#                                  0].key < start_node.key or start_node.rhs != start_node.g):
#             node = heappop(open_list)
#             k_old = node.key[0]
#             if node.g > node.rhs:
#                 node.g = node.rhs
#                 for neighbor in self.get_neighbors(node):
#                     update_node(neighbor)
#             else:
#                 node.g = inf
#                 update_node(node)
#                 for neighbor in self.get_neighbors(node):
#                     update_node(neighbor)
#             if k_old < calculate_key(start_node):
#                 heappush(open_list, node)
#             elif start_node.rhs != start_node.g:
#                 calculate_key(start_node)
#         if start_node.rhs == inf:
#             return None
#         path = []
#         node = start_node
#         while node != goal_node:
#             path.append(node)
#             node = node.parent
#         path.append(goal_node)
#         # path.reverse()
#         # poses = []
#         # for node in path:
#         #     pose = PoseStamped()
#         #     pose.header = self.header
#         #     pose.pose.position.x = node.x * msg.info.resolution
#         #     pose.pose.position.y = node.y * msg.info.resolution
#         #     pose.pose.orientation.w = 1
#         #     poses.append(pose)
#         # self.path.poses = poses
#         return path

# def publish_path(self, path):
#     self.path_pub.publish(path)
#
# def run(self):
#     rospy.spin()


if __name__ == "__main__":
    Dealer()

    rospy.spin()
