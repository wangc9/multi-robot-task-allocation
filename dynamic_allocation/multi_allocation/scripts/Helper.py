#!/usr/bin/env python3

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from multi_allocation.srv import AskWorkload
from multi_allocation.srv import AssignAuctioneer
from multi_allocation.srv import ClearAuctioneer, ClearAuctioneerResponse
from multi_allocation.srv import MainTaskAllocation, MainTaskAllocationRequest
from multi_allocation.srv import OuterSwap
from std_srvs.srv import Trigger, TriggerResponse


class Helper:

    def __init__(self):
        rospy.init_node('helper')
        self.robots = []
        self.tasks = []
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

    def listener(self):
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

    # def register_distance(self, map_data):
    #     """
    #     costmap callback
    #     :return:
    #     """
    #     self.map = map_data
    #     self.width = self.map.info.width
    #     self.height = self.map.info.height
    #     self.map_np = np.array(self.map.data).reshape((self.width, self.height))
    #     self.resolution = self.map.info.resolution
    #     self.distances_dict = self.dijkstra(self.map_np, self.resolution)
    #
    # def dijkstra(self, costmap_np, resolution):
    #     rows, cols = costmap_np.shape
    #     d_np = {}
    #     graph = {}
    #     for row in range(rows):
    #         for col in range(cols):
    #             if 0 <= costmap_np[row, col] < 100:
    #                 node = (row, col)
    #                 neighbors = self.get_neighbors(node, costmap_np)
    #                 graph[node] = {neighbor: self.get_distance(node, neighbor, resolution) for neighbor in neighbors}
    #
    #     print(f'{len(graph)} calculation needed')
    #
    #     for i, start in enumerate(graph.keys()):
    #         distances = {node: np.inf for node in graph}
    #         distances[start] = 0
    #         heap = [(0, start)]
    #
    #         while heap:
    #             current_distance, current_node = heapq.heappop(heap)
    #             if current_distance > distances[current_node]:
    #                 continue
    #             for neighbor, distance in graph[current_node].items():
    #                 tentative_distance = distances[current_node] + distance
    #                 if tentative_distance < distances[neighbor]:
    #                     distances[neighbor] = tentative_distance
    #                     heapq.heappush(heap, (tentative_distance, neighbor))
    #
    #         d_np[start] = distances
    #         print(f'{i} finished')
    #
    #     return d_np
    #
    # def get_neighbors(self, node, costmap):
    #     rows, cols = costmap.shape
    #     row, col = node
    #     neighbors = []
    #     for d_row in [-1, 0, 1]:
    #         for d_col in [-1, 0, 1]:
    #             if d_row == 0 and d_col == 0:
    #                 continue
    #             new_row = row + d_row
    #             new_col = col + d_col
    #             if 0 <= new_row < rows and 0 <= new_col < cols and 0 <= costmap[new_row, new_col] < 100:
    #                 neighbors.append((new_row, new_col))
    #
    #     return neighbors
    #
    # def get_distance(self, node_1, node_2, resolution):
    #     x1, y1 = node_1
    #     x2, y2 = node_2
    #     dx = (x1 - x2) * resolution
    #     dy = (y1 - y2) * resolution
    #
    #     return np.sqrt(dx**2 + dy**2)

    # def map_callback(self, data):
    #     # Store the map and its properties
    #     self.map = data
    #     self.resolution = data.info.resolution
    #     self.width = data.info.width
    #     self.height = data.info.height
    #     print(f'resolution: {self.resolution}\nwidth: {self.width}\nheight: {self.height}')
    #
    # def a_star(self, start, goal):
    #     # # Store the robot's position as the start position
    #     # self.start = (start,goal)
    #     #
    #     # # Set the goal position
    #     # self.goal = (self.width - 1, self.height - 1)
    #
    #     # Plan the path
    #     path, cost = self.plan_path(start, goal)
    #
    #     # Publish the path as a ROS Path message
    #     path_msg = Path()
    #     path_msg.header = Header()
    #     path_msg.header.stamp = rospy.Time.now()
    #     # print(f'path: {path}')
    #     for point in path:
    #         # print(f'point: {point}')
    #         pose = PoseStamped()
    #         pose.header = path_msg.header
    #         pose.pose.position = Point(
    #             point[0] * self.resolution + self.map.info.origin.position.x,
    #             point[1] * self.resolution + self.map.info.origin.position.y, 0)
    #         pose.pose.orientation = Quaternion(0, 0, 0, 1)
    #         path_msg.poses.append(pose)
    #
    #     return path_msg, cost
    #
    # def plan_path(self, start, goal):
    #     # Initialize the open and closed sets
    #     open_set = {start: self.heuristic(start, goal)}
    #     closed_set = set()
    #     came_from = {}
    #     g_score = {start: 0.0}
    #     f_score = {start: self.heuristic(start, goal)}
    #
    #     # Initialize the costs dictionary to store the cost of each node
    #     costs = {start: 0.0}
    #
    #     while open_set:
    #         # Get the node in the open set with the lowest f score
    #         current = min(open_set, key=open_set.get)
    #
    #         # Check if the current node is the goal node
    #         if current == goal:
    #             # Construct the path and return it
    #             path = [goal]
    #             total_cost = 0.0
    #             while path[-1] != start:
    #                 total_cost += costs[path[-1]]
    #                 path.append(came_from[path[-1]])
    #             path.reverse()
    #             return path, total_cost
    #
    #         # Move the current node from the open set to the closed set
    #         open_set.pop(current)
    #         closed_set.add(current)
    #
    #         # Get the neighbors of the current node
    #         neighbors = self.get_neighbors(current)
    #
    #         for neighbor in neighbors:
    #             if neighbor in closed_set:
    #                 continue
    #
    #             tentative_g_score = g_score[current] + self.get_cost(current,
    #                                                                  neighbor)
    #
    #             if neighbor not in g_score or tentative_g_score < g_score[
    #                 neighbor]:
    #                 g_score[neighbor] = tentative_g_score
    #                 f_score[neighbor] = g_score[neighbor] + self.heuristic(
    #                     neighbor, goal)
    #                 came_from[neighbor] = current
    #                 costs[neighbor] = costs[current] + self.get_cost(current,
    #                                                                  neighbor)
    #
    #                 if neighbor not in open_set:
    #                     open_set[neighbor] = f_score[neighbor]
    #
    #     # If the open set is empty and the goal node was not found, return None
    #     return None, None
    #
    # def get_neighbors(self, node):
    #     # Get the neighbors of the current node
    #     neighbors = []
    #     for i in range(-1, 2):
    #         for j in range(-1, 2):
    #             # Skip the current node and diagonal neighbors
    #             if i == 0 and j == 0:
    #                 continue
    #             if abs(i) + abs(j) > 1:
    #                 continue
    #             # Get the neighbor's position
    #             x = node[0] + i
    #             y = node[1] + j
    #             # Skip the neighbor if it is outside the map
    #             if x < 0 or x >= self.width or y < 0 or y >= self.height:
    #                 continue
    #             # Skip the neighbor if it is an obstacle
    #             if self.map.data[y * self.width + x] > 50:
    #                 continue
    #             # Add the neighbor to the list of neighbors
    #             neighbors.append((x, y))
    #     return neighbors
    #
    # def get_cost(self, node1, node2):
    #     # Calculate the cost to move from node1 to node2
    #     x_diff = node2[0] - node1[0]
    #     y_diff = node2[1] - node1[1]
    #     if abs(x_diff) + abs(y_diff) == 1:
    #         # Straight move
    #         return 1
    #     elif abs(x_diff) == 1 and abs(y_diff) == 1:
    #         # Diagonal move
    #         return math.sqrt(2)
    #     else:
    #         # Invalid move
    #         return float('inf')
    #
    # def heuristic(self, node, goal):
    #     # Calculate the estimated cost to the goal using the Euclidean distance heuristic
    #     return math.sqrt((goal[0] - node[0]) ** 2 + (goal[1] - node[1]) ** 2)
    #
    # def calculate_path_length(self, path):
    #     first_time = True
    #     prev_x = 0.0
    #     prev_y = 0.0
    #     total_distance = 0.0
    #     for current_point in path.poses:
    #         x = current_point.pose.position.x
    #         y = current_point.pose.position.y
    #         if not first_time:
    #             total_distance += math.hypot(prev_x - x, prev_y - y)
    #         else:
    #             first_time = False
    #         prev_x = x
    #         prev_y = y
    #
    #     return total_distance
    #
    # def register_distance(self):
    #     ogm = OccupancyGridManager("/tb3_0/move_base/global_costmap/costmap",
    #                                subscribe_to_updates=False)
    #     available_grid = []
    # #     x = range(ogm.width)
    # #     y = range(ogm.height)
    #     for i in range(self.width):
    #         for j in range(self.height):
    #             if ogm.get_cost_from_costmap_x_y(i, j) != 100:
    #                 available_grid.append((i, j))
    #     self.count = len(available_grid)
    #     u = range(len(available_grid))
    #     seq = 0
    #     seq_2 = 0
    #     for a in u:
    #         for b in range(a+1, len(available_grid)):
    #             a_x, a_y = available_grid[a][0], available_grid[a][1]
    #             b_x, b_y = available_grid[b][0], available_grid[b][1]
    #             start = (a_x, a_y)
    #             goal = (b_x, b_y)
    #             temp_path, cost = self.a_star(start, goal)
    # #             rospy.wait_for_service(f'/tb3_X/move_base/make_plan')
    # #             get_plan = rospy.ServiceProxy(f'/tb3_X/move_base/make_plan',
    # #                                           GetPlan)
    # #             req = GetPlanRequest()
    # #             start = PoseStamped()
    # #             end = PoseStamped()
    # #             start.header.seq = seq
    # #             start.header.frame_id = "map"
    # #             start.header.stamp = rospy.Time.now()
    # #             start.pose.position.x = a_x
    # #             start.pose.position.y = a_y
    # #             start.pose.position.z = 0.0
    # #             start.pose.orientation.x = 0.0
    # #             start.pose.orientation.y = 0.0
    # #             start.pose.orientation.z = 0.0
    # #             start.pose.orientation.w = 0.0
    # #
    # #             end.header.seq = seq_2
    # #             end.header.frame_id = "map"
    # #             end.header.stamp = rospy.Time.now()
    # #             end.pose.position.x = b_x
    # #             end.pose.position.y = b_y
    # #             end.pose.position.z = 0.0
    # #             end.pose.orientation.x = 0.0
    # #             end.pose.orientation.y = 0.0
    # #             end.pose.orientation.z = 0.0
    # #             end.pose.orientation.w = 0.0
    # #
    # #             req.start = start
    # #             req.goal = end
    # #             req.tolerance = 0.5
    # #
    # #             temp_plan = get_plan(req)
    # #             dist = self.calculate_path_length(temp_plan.plan)
    #             self.distances[(a_x, a_y)] = self.distances.get((a_x, a_y), dict())
    #             self.distances[(a_x, a_y)][(b_x, b_y)] = cost
    #             seq_2 += 1
    #             print(f'{self.count} left to calculate...')
    #             self.count -= 1
    #         seq_2 = 0
    #         seq += 1
    #     print(self.distances)


if __name__ == '__main__':
    helper = Helper()
    helper.ask_auctioneer()
    helper.listener()

    rospy.spin()
