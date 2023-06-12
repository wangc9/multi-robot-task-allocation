import math
import time
from typing import List

import factorgraph as fg
import networkx as nx
import numpy as np
from shapely.geometry import Polygon, Point
from sklearn.preprocessing import normalize

import rosnode
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from multi_allocation.srv import ChangeDoorLayer, ChangeDoorLayerRequest
from multi_allocation.srv import EstimateDistance, EstimateDistanceResponse
from multi_allocation.srv import UpdateFactor, UpdateFactorRequest
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class RoomBuilder:

    def __init__(self):
        self.world_frame = "map"
        rospy.init_node("door_node")
        self.robot_list = []
        names = rosnode.get_node_names()
        for name in names:
            if "tb3" in name and "move_base" in name:
                self.robot_list.append(f'/{name.split("/")[1]}')
        self.id = f'/{rospy.get_name().split("/")[1]}'
        self.robot_list.remove(self.id)
        rospy.loginfo(f'{self.id}: {self.robot_list}')
        self.points = dict()
        self.doors = dict()
        self.rooms = dict()
        self.room_poly = dict()
        self.positions = dict()
        self.distances = None
        self.room_graph = nx.Graph()
        self.factor_graph = fg.Graph()
        self.counter_1 = 0
        self.counter_2 = 0
        self.read_points()
        rospy.wait_for_service(f'{self.id}/change_door_layer_service')
        self.update_door_map_service = rospy.ServiceProxy(
            f'{self.id}/change_door_layer_service', ChangeDoorLayer)
        self.update_factor_service = rospy.Service(f'{self.id}/update_factor',
                                                   UpdateFactor,
                                                   self.update_factor_callback)
        rospy.Subscriber(f"{self.id}/amcl_pose", PoseWithCovarianceStamped,
                         self.move_callback)
        self.distance_estimator()

    def world_to_grid(self, point):
        x = point[0] + 21.2
        y = point[1] + 10.0
        row = int(round(y / 0.05000000074505806))
        col = int(round(x / 0.05000000074505806))

        return [col, row]

    def calculate_distance(self, point_1, point_2):
        return np.sqrt(
            (point_2[0] - point_1[0]) ** 2 + (point_2[1] - point_1[1]) ** 2)

    def distance_est_feedback(self, request):
        tic = time.perf_counter()
        response = EstimateDistanceResponse()
        start_point = (
            request.start.pose.position.x, request.start.pose.position.y)
        end_point = (request.end.pose.position.x, request.end.pose.position.y)
        room_start = self.room_finder(request.start)
        room_end = self.room_finder(request.end)
        if room_start == room_end:
            response.distance = self.calculate_distance(start_point, end_point)
        else:
            start_dist = self.calculate_distance(
                self.room_graph.nodes[room_start]["centre"], start_point)
            topo_dist = self.distances[room_start][room_end]
            end_dist = self.calculate_distance(
                self.room_graph.nodes[room_end]["centre"], end_point)
            response.distance = start_dist + topo_dist + end_dist
        toc = time.perf_counter()
        print(f"Calculated distance in {toc - tic:0.4f} seconds")

        return response

    def distance_estimator(self):
        distance_est_service = rospy.Service(f'{self.id}/distance_estimator',
                                             EstimateDistance,
                                             self.distance_est_feedback)

    def read_points(self):
        add_room = False
        connect_room = False
        neglect = ["---", "header:", "stamp:", "secs:", "nsecs:", "frame_id:",
                   "point:", "z:"]
        counter_1 = 0
        counter_2 = 0
        points = []
        with open(
                "/home/charles/catkin_ws/src/dynamic_allocation"
                "/multi_allocation/door.txt") as e:
            for line in e:
                if any([t in line for t in neglect]):
                    continue
                if "x:" in line:
                    x = float(line.split()[-1])
                if "y:" in line:
                    y = float(line.split()[-1])
                    c_point = self.world_to_grid([x, y])
                    points.append(c_point)
                    counter_1 += 1
                    if counter_1 == 2:
                        counter_1 = 0
                        self.doors[counter_2] = points
                        points = []
                        counter_2 += 1

        with open(
                "/home/charles/catkin_ws/src/dynamic_allocation"
                "/multi_allocation/room.txt") as f:
            for line in f:
                if any([t in line for t in neglect]):
                    continue
                if "seq" in line:
                    i = int(line.split()[-1])
                if "x:" in line:
                    x = float(line.split()[-1])
                if "y:" in line:
                    y = float(line.split()[-1])
                    self.points[i] = (x, y)
                if add_room:
                    if "******" in line:
                        add_room = False
                        connect_room = True
                        continue
                    temp = line.split()
                    temp = [int(a) for a in temp]
                    self.rooms[temp[0]] = [self.points[i] for i in temp[1:]]
                    self.room_poly[temp[0]] = Polygon(self.rooms[temp[0]])
                    centre_x = 0.25 * sum(
                        point[0] for point in self.rooms[temp[0]])
                    centre_y = 0.25 * sum(
                        point[1] for point in self.rooms[temp[0]])
                    self.rooms[temp[0]].append(centre_x)
                    self.rooms[temp[0]].append(centre_y)
                    self.room_graph.add_node(temp[0],
                                             points=self.rooms[temp[0]],
                                             centre=(centre_x, centre_y))
                    self.positions[temp[0]] = (centre_x, centre_y)
                if "room" in line:
                    add_room = True
                if connect_room:
                    content = line.split()
                    parent = int(content[0])
                    for child in content[1:]:
                        child_split = child.split('|')
                        child_node = int(child_split[0][:-1])
                        door = True if child_split[0][-1] == 'T' else False
                        if len(child_split) > 1:
                            door_pos = self.doors[int(child_split[1])]
                        else:
                            door_pos = None
                        distance = self.calculate_distance(
                            self.rooms[parent][-2:],
                            self.rooms[child_node][-2:])
                        real_dist = distance
                        self.room_graph.add_edge(parent, child_node, door=door,
                                                 distance=distance,
                                                 real_dist=real_dist, open=1,
                                                 close=0, door_pos=door_pos,
                                                 factor=np.array(
                                                     [[1, 1], [1, 1]]))
                        if door:
                            parent_rv = content[0]
                            child_rv = child_split[0][:-1]
                            initial_array = np.array([[1, 1], [1, 1]])
                            norm_array = normalize(initial_array, axis=1,
                                                   norm='l1')
                            if not self.factor_graph.has_rv(parent_rv):
                                self.factor_graph.rv(parent_rv, 2)
                            if not self.factor_graph.has_rv(child_rv):
                                self.factor_graph.rv(child_rv, 2)
                            if parent < child_node:
                                self.factor_graph.factor([parent_rv, child_rv],
                                                         potential=norm_array)
                            else:
                                self.factor_graph.factor([child_rv, parent_rv],
                                                         potential=norm_array)
        self.factor_graph.factor(['10', '15'], potential=norm_array)
        self.factor_graph.factor(['10', '17'], potential=norm_array)

        # print("edges:", self.room_graph.edges.data("door_pos"))
        # print("nodes:", self.room_graph.nodes.data("centre"))
        self.distances = dict(
            nx.shortest_path_length(self.room_graph, weight="distance"))
        # print(self.distances)
        # nx.draw(self.room_graph, pos=self.positions, with_labels=True,
        #         font_weight='bold')
        # nx.draw_networkx_edge_labels(self.room_graph, pos=self.positions)
        # plt.show()

    def room_finder(self, pose_msg):
        point = Point(pose_msg.pose.position.x, pose_msg.pose.position.y)
        for zone_id, zone in self.room_poly.items():
            if zone.contains(point):
                return zone_id
        return None

    def update_layer(self):
        marginals = self.factor_graph.rv_marginals(normalize=True)
        propability = {str(marg[0]): marg[1][1] for marg in marginals}
        useful = ['0', '1', '2', '6', '7', '11', '12', '13', '15', '17', '23',
                  '26', '28', '31']
        u_propability = {key: propability[key] for key in useful}
        # print(u_propability)
        request = ChangeDoorLayerRequest()
        for door in u_propability:
            # print(door, type(door))
            for neighbor in self.room_graph.neighbors(int(door)):
                if self.room_graph[int(door)][neighbor]['door']:
                    door_pos = self.room_graph[int(door)][neighbor]['door_pos']
                    self.room_graph[int(door)][neighbor]['real_dist'] += \
                        propability[door] * 50
                    request.start = door_pos[0]
                    request.end = door_pos[1]
                    request.cost = int(100 * propability[door])
                    result = self.update_door_map_service(request)
                    # print(f'{door} updated')
        self.distances = dict(
            nx.shortest_path_length(self.room_graph, weight="real_dist"))

    def log_marginals(self):
        """
        Print out the current marginals of all nodes in the factor graph after \
        every 50 updates
        :return: None
        """
        self.counter_1 += 1
        if self.counter_1 == 50:
            self.factor_graph.print_rv_marginals(normalize=True)
            self.counter_1 = 0

    def call_factor_update(self, s_room: int, e_room: int, index: List[int]):
        """
        Call service `{robot}/update_factor to update the factor of the robot \
        in its `self.room_graph`
        :param s_room: number of the room of start
        :param e_room: number of the room of end
        :param index: A (1, 2) array indicating which item in the factor array \
        needs to change
        :return: None. Error will be printed out in the console
        """
        request = UpdateFactorRequest()
        request.s_room = s_room
        request.e_room = e_room
        request.index = index
        for robot in self.robot_list:
            rospy.wait_for_service(f'{robot}/update_factor')
            up_global_factor_srv = rospy.ServiceProxy(f'{robot}/update_factor',
                                                      UpdateFactor)
            result = up_global_factor_srv(request)
            if not result:
                rospy.logwarn(f'{self.id}: {robot} can not update factor')

    def update_factor_callback(self, req):
        """
        Callback for updating `self.room_graph['factor']` and `self.factor_gra\
        ph.factor`, lbp will be performed afterwords. Automatically called in \
        `{self.id}/update_factor` service
        :param req: UpdateFactorRequest (s_room: start number, e_room: end num\
         ber
        :return: True
        """
        a = self.room_graph[req.s_room][req.e_room]['factor']
        a[req.index[0]][req.index[1]] += 1
        if req.s_room < req.e_room:
            b = str(req.s_room)
            c = str(req.e_room)
        else:
            b = str(req.e_room)
            c = str(req.s_room)
        self.factor_graph.factor([b, c],
                                 potential=normalize(a, axis=1, norm='l1'))
        rospy.loginfo(f'{self.id}: received {b}-{c} factor')
        self.counter_2 += 1
        if self.counter_2 >= 10:
            self.counter_2 = 0
            iters, converged = self.factor_graph.lbp(normalize=True)
            rospy.loginfo(f'{self.id}: LBP done')
            self.update_layer()
            self.log_marginals()

        return True

    def change_door(self, zone, status):
        for neighbor in self.room_graph.neighbors(zone):
            if self.room_graph[zone][neighbor]['door']:
                if status:
                    self.room_graph[zone][neighbor]['open'] += 1
                    self.room_graph[zone][neighbor]['factor'][0][0] += 1
                    self.call_factor_update(zone, neighbor, [0, 0])
                    rospy.loginfo(f"{self.id}: {zone} --- {neighbor}")
                else:
                    self.room_graph[zone][neighbor]['close'] += 1
                    if zone < neighbor:
                        self.room_graph[zone][neighbor]['factor'][0][1] += 1
                        self.call_factor_update(zone, neighbor, [0, 1])
                    else:
                        self.room_graph[zone][neighbor]['factor'][1][0] += 1
                        self.call_factor_update(zone, neighbor, [1, 0])
                    rospy.loginfo(f"{self.id}: {zone} ||| {neighbor}")
                if zone < neighbor:
                    self.factor_graph.factor([str(zone), str(neighbor)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     neighbor][
                                                     'factor'], axis=1,
                                                 norm='l1'))
                else:
                    self.factor_graph.factor([str(neighbor), str(zone)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     neighbor][
                                                     'factor'], axis=1,
                                                 norm='l1'))

    def degree_calculator(self, a: List[float], offset: int):
        """
        Calculate the index of laser scan point north, west, south, and east \
        of map
        :param a: laser scan data
        :param offset: degree offset of robot's heading (angle between robot's \
        heading and map's west axis)
        :return: dict of laser scan index {'up': [int], 'left': [int], 'down': \
        [int], 'right': [int]}
        """
        if 0 <= offset < 30:
            right = a[:(30 - offset)] + a[(330 - offset):]
            up = a[(60 - offset):(120 - offset)]
            left = a[(150 - offset):(210 - offset)]
            down = a[(240 - offset):(300 - offset)]
        elif 30 <= offset <= 60:
            right = a[(330 - offset):(390 - offset)]
            up = a[(60 - offset):(120 - offset)]
            left = a[(150 - offset):(210 - offset)]
            down = a[(240 - offset):(300 - offset)]
        elif 60 < offset < 120:
            up = a[(420 - offset):] + a[:(120 - offset)]
            left = a[(150 - offset):(210 - offset)]
            down = a[(240 - offset):(300 - offset)]
            right = a[(330 - offset):(390 - offset)]
        elif 120 <= offset <= 150:
            left = a[(150 - offset):(210 - offset)]
            down = a[(240 - offset):(300 - offset)]
            right = a[(330 - offset):(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
        elif 150 < offset < 210:
            left = a[(510 - offset):] + a[:(210 - offset)]
            down = a[(240 - offset):(300 - offset)]
            right = a[(330 - offset):(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
        elif 210 <= offset <= 240:
            down = a[(240 - offset):(300 - offset)]
            right = a[(330 - offset):(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
            left = a[(510 - offset):(570 - offset)]
        elif 240 < offset < 300:
            down = a[(600 - offset):] + a[:(300 - offset)]
            right = a[(330 - offset):(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
            left = a[(510 - offset):(570 - offset)]
        elif 300 <= offset <= 330:
            right = a[(330 - offset):(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
            left = a[(510 - offset):(570 - offset)]
            down = a[(600 - offset):(660 - offset)]
        else:
            right = a[(690 - offset):] + a[:(390 - offset)]
            up = a[(420 - offset):(480 - offset)]
            left = a[(510 - offset):(570 - offset)]
            down = a[(600 - offset):(660 - offset)]

        result = {'up': up, 'left': left, 'down': down, 'right': right}

        return result

    def update_door(self, zone: int, angle: float):
        """
        Given the current zone of the robot and the orientation (yaw), update \
        the door status to be opened and closed. The factor matrix is also \
        updated.
        :param zone: Zone number that the robot is currently in.
        :param angle: Euler angle of yaw of the robot.
        :return: None. The status of the door is updated in `self.room_graph\
        [zone][neighbor]{['open'] / ['closed'] / ['factor']}
        """
        if angle < 0:
            deg_angle = int(360 + angle * math.pi)
        else:
            deg_angle = int(angle * math.pi)
        laser = rospy.wait_for_message(f"{self.id}/scan", LaserScan)
        laser_data = laser.ranges
        laser_subset = self.degree_calculator(laser_data, deg_angle)
        # print(self.id)
        # print(len(laser_data))
        # print("0", laser_data[0])
        # print(type(laser_data[0]))
        # print("90", laser_data[89])
        # print("180", laser_data[179])
        # print("270", laser_data[269])
        if zone in [0, 1, 2, 15, 17]:
            # if angle < -0.5 * math.pi or angle > 0.5 * math.pi:
            #     sub_data = laser_data[135:225]
            # else:
            #     sub_data = laser_data[:45] + laser_data[315:]
            # print('right', laser_subset['right'])
            if any(x > 2 or x == float('inf') for x in laser_subset['right']):
                self.change_door(zone, True)
            else:
                self.change_door(zone, False)
        elif zone in [5, 6, 7, 26, 28]:
            # if angle < -0.5 * math.pi or angle > 0.5 * math.pi:
            #     sub_data = laser_data[:45] + laser_data[315:]
            # else:
            #     sub_data = laser_data[135:225]
            # print('left', laser_subset['left'])
            if any(x > 2 or x == float('inf') for x in laser_subset['left']):
                self.change_door(zone, True)
            else:
                self.change_door(zone, False)
        elif zone in [11, 12, 13, 31]:
            # if 0 < angle < math.pi:
            #     sub_data = laser_data[135:225]
            # else:
            #     sub_data = laser_data[:45] + laser_data[315:]
            # print('down', laser_subset['down'])
            if any(x > 2 or x == float('inf') for x in laser_subset['down']):
                self.change_door(zone, True)
            else:
                self.change_door(zone, False)
        elif zone in [8, 9, 10, 23]:
            # if 0 < angle < math.pi:
            #     sub_data = laser_data[:45] + laser_data[315:]
            # else:
            #     sub_data = laser_data[135:225]
            # print('up', laser_subset['up'])
            if any(x > 2 or x == float('inf') for x in laser_subset['up']):
                self.change_door(zone, True)
            else:
                self.change_door(zone, False)
        elif zone in [3, 4]:
            # left_data = 360 - int(math.degrees(angle)) - 180
            # if left_data > 315:
            #     sub_left = laser_data[(left_data - 45):] + laser_data[
            #                                                :(360 - left_data)]
            # elif left_data < 45:
            #     sub_left = laser_data[(360 - left_data):] + laser_data[
            #                                                 :left_data]
            # else:
            #     sub_left = laser_data[(left_data - 45):(left_data + 45)]
            # right_data = (360 - int(math.degrees(angle))) % 360
            # if right_data > 315:
            #     sub_right = laser_data[(right_data - 45):] + laser_data[:(
            #             360 - right_data)]
            # elif right_data < 45:
            #     sub_right = laser_data[(360 - right_data):] + laser_data[
            #                                                   :right_data]
            # else:
            #     sub_right = laser_data[(right_data - 45):(right_data + 45)]
            if any(x > 2 or x == float('inf') for x in laser_subset['left']):
                if zone == 4:
                    self.room_graph[zone][1]['open'] += 1
                    self.room_graph[zone][1]['factor'][0][0] += 1
                    self.call_factor_update(zone, 1, [0, 0])
                    self.factor_graph.factor(['1', '4'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     1]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 4 --- 1")
                else:
                    self.room_graph[zone][0]['open'] += 1
                    self.room_graph[zone][0]['factor'][0][0] += 1
                    self.call_factor_update(zone, 0, [0, 0])
                    self.factor_graph.factor(['0', '3'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     0]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 3 --- 0")
            else:
                if zone == 4:
                    self.room_graph[zone][1]['close'] += 1
                    self.room_graph[zone][1]['factor'][1][0] += 1
                    self.call_factor_update(zone, 1, [1, 0])
                    self.factor_graph.factor(['1', '4'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     1]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 4 ||| 1")
                else:
                    self.room_graph[zone][0]['close'] += 1
                    self.room_graph[zone][0]['factor'][1][0] += 1
                    self.call_factor_update(zone, 0, [1, 0])
                    self.factor_graph.factor(['0', '3'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     0]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 3 ||| 0")
            if any(x > 2 or x == float('inf') for x in laser_subset['right']):
                if zone == 4:
                    self.room_graph[zone][7]['open'] += 1
                    self.room_graph[zone][7]['factor'][0][0] += 1
                    self.call_factor_update(zone, 7, [0, 0])
                    self.factor_graph.factor([str(zone), str(7)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     7]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 4 --- 7")
                else:
                    self.room_graph[zone][6]['open'] += 1
                    self.room_graph[zone][6]['factor'][0][0] += 1
                    self.call_factor_update(zone, 6, [0, 0])
                    self.factor_graph.factor([str(zone), str(6)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     6]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 3 --- 6")
            else:
                if zone == 4:
                    self.room_graph[zone][7]['close'] += 1
                    self.room_graph[zone][7]['factor'][0][1] += 1
                    self.call_factor_update(zone, 7, [0, 1])
                    self.factor_graph.factor([str(zone), str(7)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     7]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 4 ||| 7")
                else:
                    self.room_graph[zone][6]['close'] += 1
                    self.room_graph[zone][6]['factor'][0][1] += 1
                    self.call_factor_update(zone, 6, [0, 1])
                    self.factor_graph.factor([str(zone), str(6)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     6]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 3 ||| 6")
        elif zone in [19, 21]:
            # left_data = 360 - int(math.degrees(angle)) - 180
            # if left_data > 315:
            #     sub_left = laser_data[(left_data - 45):] + laser_data[
            #                                                :(360 - left_data)]
            # elif left_data < 45:
            #     sub_left = laser_data[(360 - left_data):] + laser_data[
            #                                                 :left_data]
            # else:
            #     sub_left = laser_data[(left_data - 45):(left_data + 45)]
            # right_data = (360 - int(math.degrees(angle))) % 360
            # if right_data > 315:
            #     sub_right = laser_data[(right_data - 45):] + laser_data[:(
            #             360 - right_data)]
            # elif right_data < 45:
            #     sub_right = laser_data[(360 - right_data):] + laser_data[
            #                                                   :right_data]
            # else:
            #     sub_right = laser_data[(right_data - 45):(right_data + 45)]
            # up_data = 360 - int(math.degrees(angle)) - 270
            # if -45 < up_data < 45:
            #     sub_up = laser_data[:(up_data + 45)] + laser_data[
            #                                            (up_data - 45):]
            # else:
            #     sub_up = laser_data[(up_data - 45):(up_data + 45)]
            # down_data = - 90 - int(math.degrees(angle))
            # if -45 < down_data < 45:
            #     sub_down = laser_data[:(down_data + 45)] + laser_data[
            #                                                (down_data - 45):]
            # else:
            #     sub_down = laser_data[(down_data - 45):(down_data + 45)]
            if any(x > 2 or x == float('inf') for x in laser_subset['left']):
                if zone == 19:
                    self.room_graph[zone][15]['open'] += 1
                    self.room_graph[zone][15]['factor'][0][0] += 1
                    self.call_factor_update(zone, 15, [0, 0])
                    self.factor_graph.factor(['15', '19'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     15]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 19 --- 15")
                else:
                    self.room_graph[zone][17]['open'] += 1
                    self.room_graph[zone][17]['factor'][0][0] += 1
                    self.call_factor_update(zone, 17, [0, 0])
                    self.factor_graph.factor(['17', '21'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     17]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 21 --- 17")
            else:
                if zone == 19:
                    self.room_graph[zone][15]['close'] += 1
                    self.room_graph[zone][15]['factor'][1][0] += 1
                    self.call_factor_update(zone, 15, [1, 0])
                    self.factor_graph.factor(['15', '19'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     15]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 19 ||| 15")
                else:
                    self.room_graph[zone][17]['close'] += 1
                    self.room_graph[zone][17]['factor'][1][0] += 1
                    self.call_factor_update(zone, 17, [1, 0])
                    self.factor_graph.factor(['17', '21'],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     17]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 21 ||| 17")
            if any(x > 2 or x == float('inf') for x in laser_subset['right']):
                if zone == 19:
                    self.room_graph[zone][28]['open'] += 1
                    self.room_graph[zone][28]['factor'][0][0] += 1
                    self.call_factor_update(zone, 28, [0, 0])
                    self.factor_graph.factor([str(zone), str(28)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     28]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 19 --- 28")
                else:
                    self.room_graph[zone][26]['open'] += 1
                    self.room_graph[zone][26]['factor'][0][0] += 1
                    self.call_factor_update(zone, 26, [0, 0])
                    self.factor_graph.factor([str(zone), str(26)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     26]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 21 --- 26")
            else:
                if zone == 19:
                    self.room_graph[zone][28]['close'] += 1
                    self.room_graph[zone][28]['factor'][0][1] += 1
                    self.call_factor_update(zone, 28, [0, 1])
                    self.factor_graph.factor([str(zone), str(28)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     28]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 19 ||| 28")
                else:
                    self.room_graph[zone][26]['close'] += 1
                    self.room_graph[zone][26]['factor'][0][1] += 1
                    self.call_factor_update(zone, 26, [0, 1])
                    self.factor_graph.factor([str(zone), str(26)],
                                             potential=normalize(
                                                 self.room_graph[zone][
                                                     26]['factor'], axis=1,
                                                 norm='l1'))
                    rospy.loginfo(f"{self.id}: 21 ||| 26")
            if zone == 19:
                if any(x > 2 or x == float('inf') for x in laser_subset['up']):
                    self.room_graph[zone][31]['open'] += 1
                    self.room_graph[zone][31]['factor'][0][0] += 1
                    self.call_factor_update(zone, 31, [0, 0])
                    rospy.loginfo(f"{self.id}: 19 --- 31")
                else:
                    self.room_graph[zone][31]['close'] += 1
                    self.room_graph[zone][31]['factor'][0][1] += 1
                    self.call_factor_update(zone, 31, [0, 1])
                    rospy.loginfo(f"{self.id}: 19 ||| 31")
                self.factor_graph.factor([str(zone), str(31)],
                                         potential=normalize(
                                             self.room_graph[zone][
                                                 31]['factor'], axis=1,
                                             norm='l1'))
            if zone == 21:
                if any(x > 2 or x == float('inf') for x in
                       laser_subset['down']):
                    self.room_graph[zone][23]['open'] += 1
                    self.room_graph[zone][23]['factor'][0][0] += 1
                    self.call_factor_update(zone, 23, [0, 0])
                    rospy.loginfo(f"{self.id}: 21 --- 23")
                else:
                    self.room_graph[zone][23]['close'] += 1
                    self.room_graph[zone][23]['factor'][0][1] += 1
                    self.call_factor_update(zone, 23, [0, 1])
                    rospy.loginfo(f"{self.id}: 21 ||| 23")
                self.factor_graph.factor([str(zone), str(23)],
                                         potential=normalize(
                                             self.room_graph[zone][
                                                 23]['factor'], axis=1,
                                             norm='l1'))
        else:
            pass

        self.counter_2 += 1
        if self.counter_2 >= 10:
            iters, converged = self.factor_graph.lbp(normalize=True)
            self.update_layer()
            self.log_marginals()

    def move_callback(self, pose_co):
        pose_msg = PoseStamped()
        pose_msg.header = pose_co.header
        pose_msg.pose = pose_co.pose.pose
        orientation = pose_msg.pose.orientation
        zone = self.room_finder(pose_msg)
        rospy.loginfo(f"{self.id}: in room {zone}")
        angle = euler_from_quaternion([orientation.x, orientation.y,
                                       orientation.z, orientation.w])[2]
        rospy.loginfo(f"{self.id}: angle is {angle}")
        self.update_door(zone, angle)


if __name__ == '__main__':
    RoomBuilder()
    rospy.spin()
