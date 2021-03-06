#!/usr/bin/env python3
import rospy, roslaunch
import os
from enum import Enum
from std_msgs.msg import UInt8


class Start_routine():
    def __init__(self):
        # Enum with launch arguments
        self.Launcher = Enum('Launcher',
                             'launch_detect_lane launch_traffic_light launch_intersection launch_construction '
                             'launch_parking launch_level_crossing launch_tunnel')
        self.Arguments = Enum('Arguments', 'intersection construction parking level_crossing tunnel')

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        # package turtlebot3_autorace_2020
        self.ros_autorace2020_package_path = os.path.dirname(os.path.realpath(__file__))
        self.ros_autorace2020_package_path = self.ros_autorace2020_package_path.replace('autorace_host/src',
                                                                                        'turtlebot3_autorace_2020/')
        # package 'autorace_host'
        self.ros_autorace_host_package_path = os.path.dirname(os.path.realpath(__file__))
        self.ros_autorace_host_package_path = self.ros_autorace_host_package_path.replace('autorace_host/src',
                                                                                          'autorace_host/')
        # Traffic light launch file
        self.traffic_light_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [
            self.ros_autorace2020_package_path + "turtlebot3_autorace_detect/launch/detect_traffic_light.launch"])

        # Detect lane launch file
        self.detect_lane_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [
            self.ros_autorace_host_package_path + "/launch/detect_lane.launch"])

        self.isTrafficPublished = False
        self.max_sign_detection_value = 5

        # Boolean values for the launch status
        self.is_detect_lane_launched = False
        self.is_traffic_light_launched = False
        self.is_intersection_launched = False
        self.is_construction_launched = False
        self.is_parking_launched = False
        self.is_level_crossing_launched = False
        self.is_tunnel_launched = False

        # Boolean values for sign detection status
        self.is_intersection_finished = False
        self.is_construction_finished = False
        self.is_parking_finished = False
        self.is_level_crossing_finished = False
        self.is_tunnel_finished = False

        self.sign_detection_counter = 0

        # Start routine
        self.launch_mission(self.Launcher.launch_traffic_light.value, True)

    def start_detect_sign_launch_mission_with_arguments(self, argument, is_start):
        """ Starts the different detect sign launch files depending on the given argument """
        if is_start:
            self.cli_args = [
                self.ros_autorace2020_package_path + 'turtlebot3_autorace_detect/launch/detect_sign.launch',
                'mission:=' + argument]
            self.roslaunch_args = self.cli_args[1:]
            self.roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0], self.roslaunch_args)]
            self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file)
            self.parent.start()
        else:
            self.parent.shutdown()

    def launch_mission(self, mission_num, is_start):
        """
        Starts/Shutdowns a node with the given mission type.
        is_start True - Starts launch file
        is_start False - Shutdowns launch file
        """
        if mission_num == self.Launcher.launch_detect_lane.value:
            if is_start:
                if not self.is_detect_lane_launched:
                    pass
                    self.is_detect_lane_launched = True
                    self.detect_lane_launch.start()  # Starts detect lane node
                else:
                    pass
            else:
                if self.is_detect_lane_launched:
                    pass
                    self.is_detect_lane_launched = False  # Shutdowns detect lane node
                    self.detect_lane_launch.shutdown()

                else:
                    pass
        elif mission_num == self.Launcher.launch_traffic_light.value:
            if is_start:
                if not self.is_traffic_light_launched:
                    pass
                    self.is_traffic_light_launched = True
                    self.traffic_light_launch.start()  # Starts traffic light node
                else:
                    pass
            else:
                if self.is_traffic_light_launched:
                    pass
                    self.is_traffic_light_launched = False
                    self.traffic_light_launch.shutdown()  # Shutdowns traffic light node

                else:
                    pass
        elif mission_num == self.Launcher.launch_intersection.value:
            if is_start:
                if not self.is_intersection_launched:
                    self.is_intersection_launched = True
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.intersection.name, True)
                else:
                    pass
            else:
                if self.is_intersection_launched:
                    self.is_intersection_launched = False
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.intersection.name, False)
                else:
                    pass
        elif mission_num == self.Launcher.launch_construction.value:
            if is_start:
                if not self.is_construction_launched:
                    self.is_construction_launched = True
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.construction.name, True)
                else:
                    pass
            else:
                if self.is_construction_launched:
                    self.is_construction_launched = False
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.construction.name, False)
                else:
                    pass
        elif mission_num == self.Launcher.launch_parking.value:
            if is_start:
                if not self.is_parking_launched:
                    self.is_parking_launched = True
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.parking.name, True)
                else:
                    pass
            else:
                if self.is_parking_launched:
                    self.is_parking_launched = False
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.parking.name, False)
                else:
                    pass
        elif mission_num == self.Launcher.launch_level_crossing.value:
            if is_start:
                if not self.is_level_crossing_launched:
                    self.is_level_crossing_launched = True
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.level_crossing.name, True)
                else:
                    pass
            else:
                if self.is_level_crossing_launched:
                    self.is_level_crossing_launched = False
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.level_crossing.name, False)
                else:
                    pass
        elif mission_num == self.Launcher.launch_tunnel.value:
            if is_start:
                if not self.is_tunnel_launched:
                    self.is_tunnel_launched = True
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.tunnel.name, True)
                else:
                    pass
            else:
                if self.is_tunnel_launched:
                    self.is_tunnel_launched = False
                    self.start_detect_sign_launch_mission_with_arguments(self.Arguments.tunnel.name, False)
                else:
                    pass

    def detect_traffic_light_controller(self, msg):
        """Starts the first nodes after the green traffic light phase has been detected"""
        if not self.isTrafficPublished:
            self.launch_mission(self.Launcher.launch_detect_lane.value, True)  # Start detect lane node
            self.launch_mission(self.Launcher.launch_traffic_light.value, False)  # kill traffic light node
            self.launch_mission(self.Launcher.launch_intersection.value, True)  # Start intersection node
            self.isTrafficPublished = True

    def detect_traffic_sign_controller(self, msg):
        """Control the launched nodes"""
        if not self.is_intersection_finished:
            if msg.data == 2 or msg.data == 3:  # 2 = left sign; 3 = right sign
                self.sign_detection_counter += 1  # count how often the sign was detected
                if self.sign_detection_counter >= self.max_sign_detection_value:
                    self.is_intersection_finished = True
                    self.sign_detection_counter = 0  # reset counter
                    self.launch_mission(self.Launcher.launch_intersection.value, False)  # kill intersection node
                    rospy.sleep(62)  # wait 62 seconds
                    self.launch_mission(self.Launcher.launch_construction.value, True)  # Start construction node
        elif not self.is_construction_finished:
            if msg.data == 1:  # 1 = construction sign value
                self.sign_detection_counter += 1  # count how often the sign was detected
                if self.sign_detection_counter > self.max_sign_detection_value:
                    self.is_construction_finished = True
                    self.sign_detection_counter = 0  # reset counter
                    self.launch_mission(self.Launcher.launch_construction.value, False)  # kill construction node
                    rospy.sleep(53)  # wait 53 seconds
                    self.launch_mission(self.Launcher.launch_parking.value, True)  # Start parking node
        elif not self.is_parking_finished:
            if msg.data == 1:
                self.sign_detection_counter += 1  # count how often the sign was detected
                if self.sign_detection_counter > self.max_sign_detection_value:
                    self.is_parking_finished = True
                    self.sign_detection_counter = 0  # reset counter
                    self.launch_mission(self.Launcher.launch_parking.value, False)  # kill parking node
                    rospy.sleep(60)  # wait 60 seconds
                    self.launch_mission(self.Launcher.launch_level_crossing.value, True)  # Start level crossing node
        elif not self.is_level_crossing_finished:
            if msg.data == 1:
                self.sign_detection_counter += 1  # count how often the sign was detected
                if self.sign_detection_counter > self.max_sign_detection_value:
                    self.is_level_crossing_finished = True
                    self.sign_detection_counter = 0  # reset counter
                    self.launch_mission(self.Launcher.launch_level_crossing.value, False)  # kill level crossing node
                    rospy.sleep(70)  # wait 70 seconds
                    self.launch_mission(self.Launcher.launch_tunnel.value, True)  # Start tunnel node
        elif not self.is_tunnel_finished:
            if msg.data == 1:
                self.sign_detection_counter += 1  # count how often the sign was detected
                if self.sign_detection_counter > self.max_sign_detection_value:
                    self.is_tunnel_finished = True
                    self.sign_detection_counter = 0  # reset counter
                    self.launch_mission(self.Launcher.launch_tunnel.value, False)  # kill tunnel node
                    self.launch_mission(self.Launcher.launch_detect_lane.value, False)  # kill detect lane node

    def subscriber(self):
        rospy.Subscriber('/detect/traffic_light', UInt8, self.detect_traffic_light_controller, queue_size=1)
        rospy.Subscriber('/detect/traffic_sign', UInt8, self.detect_traffic_sign_controller, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('start')
    Start_routine().subscriber()
