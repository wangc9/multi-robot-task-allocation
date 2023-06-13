import numpy as np

import rospy
from geometry_msgs.msg import Pose
from multi_allocation.srv import ChangeDoorLayer
from nav_msgs.msg import OccupancyGrid


class MapUpdater:
    def __init__(self):
        rospy.init_node('map_updater')
        self.id = f'/{rospy.get_name().split("/")[1]}'
        temp = []
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = "map"
        self.costmap.header.stamp = rospy.Time.now()
        self.costmap.info.map_load_time = rospy.Time.now()
        self.costmap.info.width = 832
        self.costmap.info.height = 384
        self.costmap.info.origin = Pose()
        self.costmap.info.origin.position.x = -21.2
        self.costmap.info.origin.position.y = -10.0
        self.costmap.info.origin.position.z = -0.0
        self.costmap.info.origin.orientation.w = 1.0
        self.costmap.info.resolution = 0.05000000074505806
        for i in range(832):
            for j in range(384):
                temp.append(-1)
        self.costmap.data = temp
        self.np_data = np.array(self.costmap.data, dtype=np.int8).reshape(384,
                                                                          832)
        self.update_publisher = rospy.Publisher(f'{self.id}/door_map',
                                                OccupancyGrid, queue_size=10)
        self.add_door_service = rospy.Service(
            f'{self.id}/change_door_layer_service', ChangeDoorLayer,
            self.change_door_callback)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # rospy.loginfo('Prepared to add obstacle')
            # self.update_costmap(self.c_start, self.c_width, self.c_height)
            self.update_publisher.publish(self.costmap)
            rate.sleep()
            # rospy.sleep(10)
            # rospy.loginfo('Prepared to delete obstacle')
            # self.clear_costmap(self.c_start, self.c_width, self.c_height)

    def change_door_callback(self, req):
        start = req.start
        # c_start = self.world_to_grid(start)
        end = req.end
        # c_end = self.world_to_grid(end)
        cost = req.cost
        for i in range(start[0], end[0] + 1):
            for j in range(start[1], end[1] + 1):
                self.np_data[j][i] = cost
        self.costmap.data = self.np_data.reshape(1, -1)[0].tolist()

        return True

    def world_to_grid(self, point):
        x = point[0] + 21.2
        y = point[1] + 10.0
        row = int(round(y / 0.05000000074505806))
        col = int(round(x / 0.05000000074505806))

        return (col, row)


if __name__ == '__main__':
    MapUpdater()
    rospy.spin()
