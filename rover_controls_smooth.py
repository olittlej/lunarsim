from omni.kit.scripting import BehaviorScript
from pxr import Gf, UsdGeom
from omni.physx.scripts.utils import setRigidBody
import math
import time
import socket
import struct
import csv
import time
import omni.usd

#########################
# Purpose: Rover object behavior script, it 1) controls the rover to follow a set of waypoints, 2) writes state
#           information to a csv, and 3) communicates with a TCP server (sends its position/orientation, receives 
#           rock information for generation)
# Status: Complete, see server.py for server side of TCP connection, current issues with wheels colliders gripping terrain (view colliders)
# Author: Olin Littlejohn (8-16-23)
#########################

global first_time, position, rotation, csv_start
first_time = True
csv_start = True

# speed parameters - don't keep too tight or won't work. Parameters that work well: 100,80,60,70,60,10
min_dist = 100 # distance from endpoint before moving to next one
max_speed = 90 # max linear speed
min_speed = 80 # min linear speed
max_turn = 80 # max turn speed
min_turn = 70 # min turn speed
max_deviation = 10 # max angle deviation current to target

# behavior script portion
class WheelTest(BehaviorScript):
    def on_init(self):
        print("QINFO: INITIALIZE ---------------------------------------")
        global position, rotation

        # get necessary primitives based on prim paths for controls
        self._prim = self.stage.GetPrimAtPath(self.prim_path)
        self._wheelFR = self.stage.GetPrimAtPath('/World/CADRE_A/CADRE_Demo/FR/RevoluteJoint')
        self._wheelBR = self.stage.GetPrimAtPath('/World/CADRE_A/CADRE_Demo/BR/RevoluteJoint')
        self._wheelFL = self.stage.GetPrimAtPath('/World/CADRE_A/CADRE_Demo/FL/RevoluteJoint')
        self._wheelBL = self.stage.GetPrimAtPath('/World/CADRE_A/CADRE_Demo/BL/RevoluteJoint')
        self._chassis = self.stage.GetPrimAtPath('/World/CADRE_A/CADRE_Demo/Chassis')
        
        #=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self.path_csv = 'c:/Users/littlejoh/Desktop/omniverse/localpath.csv'
        #=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

        """ SETTING INITIAL POSITION TO KEVIN COORDINATES NEEDS WORK (coordinate systems complicated)
        self.row = 0
        with open(self.path_csv,newline='',encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                rows = list(reader)
                last_row = rows[self.row]
                point = [float(last_row[1]), float(last_row[3])]
                print(point)
                start_x, start_y = kevin_to_gcs(point[0], point[1])
        rotation_matrix = Gf.Matrix3d(0,-180,180)
        scale_matrix = Gf.Matrix3d(Gf.Vec3d(1,1,1))
        transform_matrix = Gf.Matrix4d()
        transform_matrix.SetTranslateOnly((start_x, start_y, 40))
        transform_matrix.SetRotate(rotation_matrix)
        transform_matrix.SetScale(scale_matrix)
        omni.usd.set_world_transform_matrix(self._prim, transform_matrix)"""

    def on_destroy(self):
        print("QINFO: DESTROY ---------------------------------------")

    def on_play(self):
        print("QINFO: PLAY ---------------------------------------")
        global position, rotation, first_time

        # for first run through
        if first_time == True:
            self.row = 1
            with open(self.path_csv,newline='',encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                rows = list(reader)
                last_row = rows[self.row]
                point_x, point_y = [float(last_row[1]), float(last_row[3])]
                #point_x, point_y = kevin_to_gcs(point[0], point[1]) for coordinate conversion if necessary. In this demo, asssumed given coordinates in omniverse coordinates already
                self.endpoint = [point_x, point_y]

                world_transform = omni.usd.get_world_transform_matrix(self._chassis)
                position = world_transform.ExtractTranslation()
                position = list(position)
                position = [position[0], position[1], position[2]]
                rotation = world_transform.ExtractRotation()
                rotation = rotation.GetQuaternion()
                rotation = [rotation.GetReal(), rotation.GetImaginary()[0], rotation.GetImaginary()[1], rotation.GetImaginary()[2]]

                P1 = [position[0], position[1]]
                P2 = self.endpoint
                self.max_dist = math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)
                self.max_max = self.max_dist
                print(f'RR max dist: {self.max_dist:.2f}')
                current_angle, roll_angle = rotation_about_z_axis(rotation)
                target_angle = angle_about_z_axis(P1, P2)
                self.max_angle = abs(current_angle - target_angle)
                print(f'RR max angle: {self.max_angle:.2f}')

            self.row += 1
            first_time = False
        self.past_typ = '-1'

    def on_pause(self):
        print("QINFO: PAUSE ---------------------------------------")

    def on_stop(self):
        print("QINFO: STOP ---------------------------------------")
        global first_time
        first_time = True

    def on_update(self, current_time: float, delta_time: float):
        #print("QINFO: UPDATE ---------------------------------------")
        global position, rotation
        global min_dist, min_speed, max_speed, max_turn, min_turn, max_deviation

        # convert local coordinates to global (use chassis since the whole rover Xform doesn't change when the rover moves)
        world_transform = omni.usd.get_world_transform_matrix(self._chassis)
        position = world_transform.ExtractTranslation()
        position = list(position)
        position = [position[0], position[1], position[2]]
        rotation = world_transform.ExtractRotation()
        rotation = rotation.GetQuaternion()
        rotation = [rotation.GetReal(), rotation.GetImaginary()[0], rotation.GetImaginary()[1], rotation.GetImaginary()[2]]

        # recorder/rock generator functions
        csv_recorder(self) # saves csv of rover states. If commented, server needs self.start_time = time.time() somewhere else in code

        # finds distance between current and target point, angle
        P1 = [position[0], position[1]]
        P2 = self.endpoint
        print(f'RINFO: {P2}')

        current_angle, roll_angle = rotation_about_z_axis(rotation)
        #print(f'RR current angle: {current_angle:.2f}')
        target_angle = angle_about_z_axis(P1, P2)
        distance = math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)
        angle_dif = abs(current_angle - target_angle)

        # smooth acceleration based on distance/angle difference, changes wheel revolutejoint target velocity
        turn_speed = (1 - 4 * ((angle_dif - max_deviation) / (self.max_angle - max_deviation) - 0.5)**2) * (max_turn - min_turn) + min_turn
        if turn_speed > max_turn:
            turn_speed = max_turn
        elif turn_speed < min_turn:
            turn_speed = min_turn
        print(f'RR angle diff: {abs(current_angle-target_angle):.2f} turn speed: {turn_speed:.2f}')
        linear_speed = (1 - 4 * ((distance - min_dist) / (self.max_dist - min_dist) - 0.5)**2) * (max_speed - min_speed) + min_speed
        if linear_speed < min_speed:
            linear_speed = min_speed
        elif linear_speed > max_speed:
            linear_speed = max_speed
        print(f'RR dist: {distance:.2f} linear speed: {linear_speed:.2f}')

        # how to proceed based on relative current/target position
        if distance <= min_dist:
            thrust_0 = 0
            thrust_1 = 0
            print("QINFO: CHECK NEW COORDINATE ---------------------------------------")
            with open(self.path_csv,newline='',encoding='utf-8-sig') as file:
                reader = csv.reader(file)
                rows = list(reader)
                last_row = rows[self.row]
                self.row += 1
                point_x, point_y = [float(last_row[1]), float(last_row[3])]
                #point_x, point_y = kevin_to_gcs(point[0], point[1])
                self.endpoint = [point_x, point_y]

                P1 = [position[0], position[1]]
                P2 = self.endpoint
                self.max_dist = math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)
                self.max_max = self.max_dist
                print(f'RR max dist: {self.max_dist:.2f}')
                current_angle, roll_angle = rotation_about_z_axis(rotation)
                target_angle = angle_about_z_axis(P1, P2)
                self.max_angle = abs(current_angle - target_angle)
                print(f'RR max angle: {self.max_angle:.2f}')

        elif (abs(current_angle - target_angle) >= max_deviation):
            print('TURN -------')
            difference = target_angle - current_angle
            if difference > 180:
                difference -= 360
            elif difference < -180:
                difference += 360
            
            if difference > 0: 
                thrust_0 = -1 * turn_speed # left side
                thrust_1 = turn_speed
            elif difference < 0:
                thrust_0 = turn_speed # left side
                thrust_1 = -1 * turn_speed
            
            self.max_dist = math.sqrt((P2[0] - P1[0])**2 + (P2[1] - P1[1])**2)
            print(f'RR max dist: {self.max_dist:.2f}')

        elif distance > min_dist:
            thrust_0 = linear_speed
            thrust_1 = linear_speed
        self._wheelFR.GetAttribute("drive:angular:physics:targetVelocity").Set(thrust_1) # these are the attributes for the rover set up with revolutejoints for wheels
        self._wheelBR.GetAttribute("drive:angular:physics:targetVelocity").Set(thrust_1) # will need to change this if the rover is rigged differently (ex. tank mode)
        self._wheelFL.GetAttribute("drive:angular:physics:targetVelocity").Set(thrust_0)
        self._wheelBL.GetAttribute("drive:angular:physics:targetVelocity").Set(thrust_0)
        print(f"THRUST L: {thrust_0:.2f} THRUST R: {thrust_1:.2f}")

# convert kevin/path coordinates to omniverse world coordinates, NOT USED in this iteration. Assumed coordinates given in spreadsheet are omniverse coordinates already
def kevin_to_gcs(kev_x, kev_y):
    # assuming omniverse world units in inches, need change for meters/cm units
    gcs_x = (kev_x * (5905.5/150)) - 5905.5
    gcs_y = (kev_y * (5905.5/150)) - 5905.5
    return gcs_x, gcs_y

# record rover time, position, orientation data into a csv
def csv_recorder(self):
    global position, rotation, csv_start
    
    if csv_start == True:
        self.start_time = time.time()
        csv_start = False

    elapsed_time = time.time() - self.start_time
    fieldnames = ['Elapsed Time','position x', 'position y', 'position z', 'rotation r', 'rotation i1', 'rotation i2', 'rotation i3']
    data = {'Elapsed Time': elapsed_time, 'position x': position[0], 'position y': position[1], 'position z': position[2], 'rotation r': rotation[0], 'rotation i1': rotation[1],
        'rotation i2': rotation[2], 'rotation i3': rotation[3]}

    with open(self.ar_csv, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if csvfile.tell() == 0:
            writer.writeheader()
        writer.writerow(data)
        #print(f"Data has been written to {self.ar_csv} successfully.")
    
    return

# below functions used for calculating the rover current and target angle about z axis
def quaternion_to_rotation_matrix(w, x, y, z):
    # Calculate the elements of the 3x3 rotation matrix
    r11 = 1 - 2*y*y - 2*z*z
    r12 = 2*x*y + 2*w*z  # Modified sign
    r13 = 2*x*z - 2*w*y  # Modified sign

    r21 = 2*x*y - 2*w*z  # Modified sign
    r22 = 1 - 2*x*x - 2*z*z
    r23 = 2*y*z + 2*w*x  # Modified sign

    r31 = 2*x*z + 2*w*y  # Modified sign
    r32 = 2*y*z - 2*w*x  # Modified sign
    r33 = 1 - 2*x*x - 2*y*y

    return [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]

def rotation_about_z_axis(quaternion):
    # Extract quaternion components
    w, x, y, z = quaternion

    # Calculate yaw angle (rotation about the z-axis)
    yaw_rad = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    yaw_deg = math.degrees(yaw_rad)

    # Normalize the angle to the range of -180 to +180 degrees
    while yaw_deg > 180:
        yaw_deg -= 360
    while yaw_deg < -180:
        yaw_deg += 360

    roll_rad = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    roll_deg = math.degrees(roll_rad)

    return yaw_deg, roll_deg

def angle_about_z_axis(point1, point2):
    # Find the directional vector from point1 to point2
    directional_vector = [point2[0] - point1[0], point2[1] - point1[1]]

    # Calculate the angle between the directional vector and the positive y-axis
    angle_rad = math.atan2(-directional_vector[0], directional_vector[1])
    angle_deg = math.degrees(angle_rad)

    angle_deg += 12

    if angle_deg > 180:
        angle_deg -= 360
    elif angle_deg < -180:
        angle_deg += 360
    
    return angle_deg
