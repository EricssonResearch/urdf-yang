#!/usr/bin/env python3

import sys
import math
import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs
import tf2_py
import threading
import time
import yaml
import json
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from os import walk
from rclpy.node import Node
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import Point, Pose, PointStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

marker_pos = 0
server = None
node = None
dynamic_markers = []
dynamic_marker_counter_id = 0
objectFiles = []
portDict = dict()
h_first_entry = 0
h_load_entry = 0
h_mode_last = 0
h_object_last = 0
h_remove_entry = 0
h_save_entry = 0
h_export_entry = 0
h_sensor_entry = 0
h_connect_entry = 0
h_create_entry = 0
h_destroy_entry = 0
menu_handler = MenuHandler()
# myPath = "/home/robot/urdf-yang/src"
robotic_arms = []
robotic_arm_counter_id = 0

class RoboticArm: #ososztaly
    def __init__(self, type, name, feedback):
        self.type = type
        self.markername = str()
        self.extensions = {
            'camera' : False,
            'camera_holder': False,
            'gripper': False,
        }
        self.robot = False
        self.exportmaterial = str()
        self.name = name
    
    def export(self, x, y, z):
            #beleirni a path-be a cuccokat
        parameter = '{xacro.load_yaml(initial_positions_file)}'
        self.exportmaterial = f'''
           <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
           \n<xacro:arg name="{self.name}" default="{self.name}"/>
            <xacro:arg name="ur_type_{self.name}" default="{self.type}"/>
            <xacro:arg name="tf_prefix_{self.name}" default="" />
            <xacro:arg name="joint_limit_params_{self.name}" default="$(find ur_description)/config/$(arg ur_type_{self.name})/joint_limits.yaml"/>
            <xacro:arg name="kinematics_params_{self.name}" default="$(find ur_description)/config/$(arg ur_type_{self.name})/default_kinematics.yaml"/>
            <xacro:arg name="physical_params_{self.name}" default="$(find ur_description)/config/$(arg ur_type_{self.name})/physical_parameters.yaml"/>
            <xacro:arg name="visual_params_{self.name}" default="$(find ur_description)/config/$(arg ur_type_{self.name})/visual_parameters.yaml"/>
            
            urdfattachment = <xacro:ur_robot
                            name="$(arg name)"
                            tf_prefix="$(arg tf_prefix_{self.name})"
                            parent="world"
                            joint_limits_parameters_file="$(arg joint_limit_params_{self.name})"
                            kinematics_parameters_file="$(arg kinematics_params_{self.name})"
                            physical_parameters_file="$(arg physical_params_{self.name})"
                            visual_parameters_file="$(arg visual_params_{self.name})"
                            transmission_hw_interface="$(arg transmission_hw_interface)"
                            safety_limits="$(arg safety_limits)"
                            safety_pos_margin="$(arg safety_pos_margin)"
                            safety_k_position="$(arg safety_k_position)"
                            use_fake_hardware="$(arg use_fake_hardware)"
                            fake_sensor_commands="$(arg fake_sensor_commands)"
                            sim_gazebo="$(arg sim_gazebo)"
                            sim_ignition="$(arg sim_ignition)"
                            headless_mode="$(arg headless_mode)"
                            initial_positions="${parameter}"
                            use_tool_communication="$(arg use_tool_communication)"
                            tool_voltage="$(arg tool_voltage)"
                            tool_parity="$(arg tool_parity)"
                            tool_baud_rate="$(arg tool_baud_rate)"
                            tool_stop_bits="$(arg tool_stop_bits)"
                            tool_rx_idle_chars="$(arg tool_rx_idle_chars)"
                            tool_tx_idle_chars="$(arg tool_tx_idle_chars)"
                            tool_device_name="$(arg tool_device_name)"
                            tool_tcp_port="$(arg tool_tcp_port)"
                            robot_ip="$(arg robot_ip)"
                            script_filename="$(arg script_filename)"
                            output_recipe_filename="$(arg output_recipe_filename)"
                            input_recipe_filename="$(arg input_recipe_filename)"
                            reverse_ip="$(arg reverse_ip)"
                            script_command_port="$(arg script_command_port)"
                            reverse_port="$(arg reverse_port)"
                            script_sender_port="$(arg script_sender_port)"
                            trajectory_port="$(arg trajectory_port)"
                            >
                            <origin xyz="{x} {y} {z}" rpy="0 0 0" /> <!--ide kell meg a pozi-->
                        </xacro:ur_robot>
                        <xacro:gripper_bracket parent="wrist_3_link" />
                        <xacro:onrobot_rg2ft 
                            prefix=""
                            hw_interface="hardware_interface/PositionJointInterface"
                            parent="tool0">
                            <origin xyz="0 -0.027 0.03" rpy="0 0 1.5707963" />
                        </xacro:onrobot_rg2ft>
                        <xacro:camera_holder parent="gripper_bracket_link" />
                        <xacro:sensor_d455 parent="camera_holder_link">
                            <origin xyz="-0.022 -0.039 -0.01" rpy="0 1.57 0" />
                        </xacro:sensor_d455>
                        \n
                        '''
        
        return self.exportmaterial

#Menu begins here

def enableCb(feedback):
    global menu_handler
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state==MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        node.get_logger().info('Hiding first menu entry')
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        node.get_logger().info('Showing first menu entry')
        menu_handler.setVisible(h_first_entry, False)

    menu_handler.reApply(server)
    node.get_logger().info('update')
    server.applyChanges() 

def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

    node.get_logger().info('Switching to menu entry #' + str(h_mode_last))
    menu_handler.reApply(server)
    server.applyChanges()

def deployDevice(feedback):
    global  server
    node.get_logger().info('deployDevice')
    MarkerWithMenu()
    server.applyChanges()

def robotListAppend(path, feedback):
    global robotic_arms, robotic_arm_counter_id

    path = str(path).split('/')
    patternforur = re.compile(r"^ur.*\.dae$")
    if patternforur.match(path[-1]):
            found = False
            for i in range(len(robotic_arms)):
                if robotic_arms[i].markername == feedback.marker_name:
                    found = True
                    robotic_arms[i].type = path[-3]
                    robotic_arms[i].name = f"{path[-3]}_{str(robotic_arms[i].name).split('_')[-1]}"
                    print('Full robot ' + robotic_arms[i].name)
                    
            if found == False:
                arm = RoboticArm(path[-3], f'{path[-3]}_{robotic_arm_counter_id}', feedback)
                robotic_arm_counter_id += 1
                arm.markername = feedback.marker_name
                robotic_arms.append(arm)
                print('Full robot ' + arm.name)
                

def changeDevice(feedback, path):
    global dynamic_markers, server, dynamic_marker_counter_id, h_object_last, objectFiles, node

    node.get_logger().info('changeDevice')

    menu_handler.setCheckState(h_object_last, MenuHandler.UNCHECKED)
    h_object_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_object_last, MenuHandler.CHECKED)
    node.get_logger().info('Switching to menu entry #' + str(h_object_last))
    menu_handler.reApply(server)

    int_marker = server.get(feedback.marker_name)


    int_marker_control = int_marker.controls[0]
    marker = int_marker_control.markers[0]
    
    print("feedback.menu_entry_id:" + str(feedback.menu_entry_id)) 
    
    objectPath = f'file://{path}'

    
    print(objectPath)
    patternforur = re.compile(r".*ur.{0,2}\.dae$")
    patternforabb = re.compile(r".*irb.*\.dae$")
    patternforcamera = re.compile(r".*[dl]\d{3}\.(dae|stl)$")
    if patternforur.match(objectPath) or patternforabb.match(objectPath):
        print('AAAAAAAAAAAAAAAAAAAA')
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0
    if '2f_85_gripper.dae' in objectPath:
        print('BBBBBBBBBBBBBBBBBBBBBBB')
        marker.scale.x = 3.0
        marker.scale.y = 3.0
        marker.scale.z = 3.0
    if 'intel_nuc.stl' in objectPath or 'raspberry' in objectPath or 'usb-dongle.stl' in objectPath:
        print('MEGVAAAAAAAAAAAAAAAAAAAAAAAAAN')
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
    if 'Mikrotik' in objectPath or 'TP-LINK' in objectPath or 'radio_dot.stl' in objectPath:
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
    if 'server-rack.stl' in objectPath or 'rg2ft_gripper.stl' in objectPath or 'kwr75.dae' in objectPath:
         print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCc')
         marker.scale.x = 0.001
         marker.scale.y = 0.001
         marker.scale.z = 0.001
    if 'gripper_bracket_with_screws.stl' in objectPath or 'skamera_ur_new.stl' in objectPath or 'd455.stl' in objectPath:
         marker.scale.x = 0.005
         marker.scale.y = 0.005
         marker.scale.z = 0.005
    if patternforcamera.match(objectPath) and 'd455.stl' not in objectPath:
         marker.scale.x = 5.0
         marker.scale.y = 5.0
         marker.scale.z = 5.0
        

    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = objectPath
    int_marker_control.markers[0] = marker
    int_marker.controls[0] = int_marker_control

    server.insert(int_marker, feedback_callback=lambda feedback, node=node: process_feedback(feedback, node.get_logger(), node))

    print("Marker changed")
    server.applyChanges()
    
    patternforur = re.compile(r"^ur.*\.dae$")
    filename = str(path).split('/')[-1]
    if patternforur.match(filename):
        robotListAppend(path, feedback)     #feedback.marker_name

def removeDevice(feedback):
    global dynamic_markers, server, dynamic_marker_counter_id, h_remove_entry, h_object_last

    node.get_logger().info('removeDevice')

    menu_handler.setCheckState(h_object_last, MenuHandler.UNCHECKED)
    h_object_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_object_last, MenuHandler.CHECKED)
    menu_handler.reApply(server)

    int_marker_for_delete = server.get(feedback.marker_name)
    server.erase(int_marker_for_delete.name)
    node.get_logger().info(f"{int_marker_for_delete.name} found for delete.")
    
    for (marker,int_marker) in dynamic_markers:
        if int_marker.name == int_marker_for_delete.name:
            dynamic_markers.remove((marker, int_marker))

    server.applyChanges()

def getObjectList(myPath):

    global objectFiles
    w = os.walk(myPath)
    filenames_all = []
    patternforur = re.compile(r"^ur.{0,2}\.dae$")
    patternforgripper = re.compile(r".*_gripper\.(dae|stl)$")
    patternforabb = re.compile(r"^irb.*\.dae$")
    for dirpath, dirnames, filenames in w:
        for file in filenames:
            filepath = os.path.join(dirpath, file)
            if (file.endswith('.stl') or file.endswith('.dae')) and 'interface_description' not in filepath and 'collision' not in filepath:
                    
                if patternforur.match(file):
                    filenames_all.append(file)

                elif patternforgripper.match(file):
                    filenames_all.append(file)

                elif patternforabb.match(file):
                    filenames_all.append(file)
        
                elif 'network-components' in filepath:
                    filenames_all.append(file)

                elif 'example_system_description' in filepath:
                    filenames_all.append(file)
                
                elif 'realsense2_description' in filepath and 'plug.stl' not in filepath:
                    filenames_all.append(file)
                
                elif 'kwr75.dae' in file or 'LD19.stl' in file or 'quadrotor_4.dae' in file:
                    filenames_all.append(file)
                
                
    objectFiles = filenames_all
    for i in objectFiles:
        print(i)
    return filenames_all  

def makeMenuMarker():
    global server, dynamic_markers, marker_pos, dynamic_marker_counter_id, menu_handler, node
    #interactive marker part
    int_marker = InteractiveMarker()
    dynamic_marker_counter_id = dynamic_marker_counter_id + 1
    int_marker.name = f"marker_{dynamic_marker_counter_id}"
    int_marker.header.frame_id = 'world' #int_marker.name 
    int_marker.pose.position.y = float(-3 * marker_pos)
    marker_pos +=1
    int_marker.scale = 1.0
    

    #interactive marker control part
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    #marker part
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = int_marker.scale * 0.45
    marker.scale.y = int_marker.scale * 0.45
    marker.scale.z = int_marker.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    #making the connections
    control.markers.append(marker)
    int_marker.controls.append(control)

    #new control mode with the same object
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    int_marker.controls.append(control)

    #rest
    server.insert(int_marker, feedback_callback=lambda feedback, node=node: process_feedback(feedback, node.get_logger(), node))
    dynamic_markers.append((marker, int_marker))

    menu_handler.apply(server, int_marker.name)

    server.applyChanges()

class MarkerWithMenu():
    #lehet ide eleg lenne egy connection kapcsolat, hogy mivel, hova es ennyi
    #es ha letrehozok egy kapcsolatot, akkor vegigmegyek az eszkozokon, es ha megvan, akkor annak az adott portjat is lefoglalom
    fromorto = 0
    temporaryconnection = {
        "fromdevice": None,
        "fromport": None,
        "todevice": None,
        "toport": None
    }
    def __init__(self):
        self.int_marker = InteractiveMarker()
        self.marker = Marker()
        self.menu_handler = MenuHandler()
        
        self.type = None
        self.id = int
        self.name = str
        self.connections = []
        self.ports = []
        self.port_menu_entries = []

        self.create_marker()
        self.create_menu()

    def create_menu(self):
        global objectFiles, server, h_first_entry, h_mode_last, h_deploy_entry, h_device_entry, h_load_entry, h_save_entry, h_remove_entry, h_export_entry, h_sensor_entry, h_connect_entry, myPath
        self.menu_handler.insert('Deploy device', callback=deployDevice)
        h_device_entry = self.menu_handler.insert('Change device')
        self.menu_handler.insert('Load', callback=load_markers_from_file)
        self.menu_handler.insert('Save', callback=save_markers_to_file)
        self.menu_handler.insert('Remove', callback=removeDevice)
        self.menu_handler.insert('Export', callback = exportMarkers)

        patternforur = re.compile(r"^ur.{0,2}\.dae$")
        patternforgripper = re.compile(r".*_gripper\.(dae|stl)$")
        patternforabb = re.compile(r"^irb.*\.dae$")

        h_sub_robot_entry = self.menu_handler.insert('Robot components', parent = h_device_entry)
        h_sub_network_entry = self.menu_handler.insert('Network components', parent = h_device_entry)
        h_rg2ft_entry = self.menu_handler.insert('Grippers', parent = h_sub_robot_entry)
        h_sensor_entry = self.menu_handler.insert('Sensors', parent= h_sub_robot_entry)
        h_realsense2_entry = self.menu_handler.insert('Onrobot cameras', parent=h_sub_robot_entry)
        h_holder_entry = self.menu_handler.insert('Camera holders', parent = h_sub_robot_entry)
        h_universal_robots_entry = self.menu_handler.insert('ur', parent=h_sub_robot_entry)
        h_abb_robots_entry = self.menu_handler.insert('abb', parent = h_sub_robot_entry)
        h_drone_entry = self.menu_handler.insert('drone', parent=h_sub_robot_entry)
        w = os.walk(myPath)

        for dirpath, dirnames, filenames in w:
         for object in objectFiles:
             for file in filenames:
                filepath = os.path.join(dirpath, file)
                if object in filepath:

                    if 'network-components' in filepath:
                        h_sub_device_entry = self.menu_handler.insert(object, parent = h_sub_network_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])
                    
                    if 'example_system_description' in filepath:
                            h_sub_device_entry = self.menu_handler.insert(object, parent = h_holder_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])
                    
                    if patternforgripper.match(file):
                            h_sub_device_entry = self.menu_handler.insert(object, parent = h_rg2ft_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)]) 
                    
                    if 'LD19.stl' in filepath or 'kwr75.dae' in filepath:
                              h_sub_device_entry = self.menu_handler.insert(object, parent = h_sensor_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])   
                    
                    if 'realsense2_description' in filepath:
                            h_sub_device_entry = self.menu_handler.insert(object, parent=h_realsense2_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])       
                    
                    if 'quadrotor' in filepath:
                        h_sub_device_entry = self.menu_handler.insert(object, parent=h_drone_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])
                    
                    if patternforabb.match(file):
                            h_sub_device_entry = self.menu_handler.insert(object, parent = h_abb_robots_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])
                        
                    if patternforur.match(file):
                            h_sub_device_entry = self.menu_handler.insert(object, parent = h_universal_robots_entry, callback=lambda feedback, path=filepath: [changeDevice(feedback, path), self.expand_menu(path)])

        self.menu_handler.reApply(server)
        server.applyChanges()
        return
    
    def expand_menu(self, path):
         global server, h_connect_entry, h_create_entry, h_destroy_entry
        
         if self.type != None:
             for entry in self.port_menu_entries:
                self.menu_handler.setVisible(entry, False)
             self.port_menu_entries.clear()
             self.connections.clear()
            
         if self.type == None:
            h_connect_entry = self.menu_handler.insert('Connectivity')
            h_create_entry = self.menu_handler.insert('Create Connection', parent=h_connect_entry)
            h_destroy_entry = self.menu_handler.insert('Destroy Connection', parent=h_connect_entry)

         self.define_ports_and_type(path)
         self.name = str(self.type) + '-' + str(self.id)

         for port in self.ports:
                c_entry = self.menu_handler.insert(port['id'], parent = h_create_entry, callback=lambda feedback: self.create_connection(port))
                d_entry = self.menu_handler.insert(port['id'], parent = h_destroy_entry, callback=lambda feedback: self.disconnect(port))
                self.port_menu_entries.append(c_entry)
                self.port_menu_entries.append(d_entry)
               
         
         #self.type = path
         print("Megvan a type: " + self.type)
         self.menu_handler.reApply(server)
         server.applyChanges()
         return
    
    def disconnect(self, port):
        #elso lepes: megnezzuk, hogy a kapcsolat letezik-e
        #masodik lepes: ha igen, akkor bontjuk a kapcsolatot, es a masik device adott portjat is szabadda tesszuk
        #               ha nem, akkor nem tortenik semmi
        #megjegyzes: mindket oldalon fel kell szabaditani
        for connection in self.connections:
            if connection['fromport'] == port:
                port['free'] = 'yes'
                connection['toport']['free'] = 'yes'
                connection['todevice'].connections.remove(connection)
                self.connections.remove(connection)
            
            elif connection['toport'] == port:
                port['free'] = 'yes'
                connection['fromport']['free'] = 'yes'
                connection['fromdevice'].connections.remove(connection)
                self.connections.remove(connection)
        return
    #modositas
    #kell egy kabel, na marmost ha letrejohet a kapcsolat, akkor a kabeltipust ki kell valasztani, hogy melyiket lehet valasztani
    #ha pedig bontjuk a kapcsolatot, akkor mindez eltunik
    def create_connection(self, port):
        #elso lepes: megkapjuk, hogy melyik eszkozrol csatlakozunk milyen tipusu csatlakozassal
        #masodik lepes: megkapjuk, hogy hova szeretnenk csatlakozni
        #harmadik lepes: megvizsgaljuk, hogy a kapcsolatot letre lehet e hozni, valamint, szabad-e
        #ha szabad: letrejon a kapcsolat, beallitjuk, hogy foglalt (esetleg lehet tuple-t tarolni, hogy from, to, micsoda)
        #ha nem szabad: nem jon letre a kapcsolat
        #megjegyzes: mindket oldalon meg kell vizsgalni, hogy szabad, vagy foglalt a port
        if MarkerWithMenu.fromorto == 0 and port['free'] == 'yes':
            MarkerWithMenu.temporaryconnection['fromdevice'] = self #lehet ide a self kene
            MarkerWithMenu.temporaryconnection['fromport'] = port
            MarkerWithMenu.fromorto = 1
            print("From device selected:", self.name, "Port:", port)
            

        elif MarkerWithMenu.fromorto == 1 and port['free'] == 'yes':
            MarkerWithMenu.temporaryconnection['todevice'] = self
            MarkerWithMenu.temporaryconnection['toport'] = port

            print("To device selected:", self.name, "Port:", port)


            MarkerWithMenu.temporaryconnection['fromport']['free'] = 'no'
            MarkerWithMenu.temporaryconnection['toport']['free'] = 'no'

            MarkerWithMenu.temporaryconnection['fromdevice'].connections.append(MarkerWithMenu.temporaryconnection.copy())
            MarkerWithMenu.temporaryconnection['todevice'].connections.append(MarkerWithMenu.temporaryconnection.copy())
            print('Connection created!')
            print(str(MarkerWithMenu.temporaryconnection['fromdevice']))
            print(str(MarkerWithMenu.temporaryconnection['todevice']))
            print(str(MarkerWithMenu.temporaryconnection['toport']))
            print(str(MarkerWithMenu.temporaryconnection['fromport']))
        

            MarkerWithMenu.temporaryconnection.clear()
            MarkerWithMenu.fromorto = 0
        return
    
    def disconnect_all(self):
        for connection in self.connections:
            if connection['fromdevice'] == self:
                connection['fromport']['free'] = 'yes'
                connection['toport']['free'] = 'yes'
                connection['todevice'].connections.remove(connection)
                self.connections.remove(connection)
            if connection['todevice'] == self:
                connection['toport']['free'] = 'yes'
                connection['fromport']['free'] = 'yes'
                connection['fromdevice'].connections.remove(connection)
                self.connections.remove(connection)
        return

    def define_ports_and_type(self, path):
         #beolvassuk a type altal leirt fajlbol a portokat, ezt elhelyezzuk egy listaban
         #megnezzuk network-komponens e, ott egyszerubb a dolgunk
         #a tobbinel meg eppen amelyik robot meglesz
         self.ports.clear()
         self.disconnect_all()

         if 'network-components' in path:
            modified_path = os.path.dirname(os.path.dirname(path))
            urdf_path = os.path.join(modified_path, "urdf")
            for file in os.listdir(urdf_path):
                if file.endswith(".urdf.xacro"):
                    xacro_file = os.path.join(urdf_path, file)
        
                    # XML betöltése és a gyökérelem megszerzése
                    tree = ET.parse(xacro_file)
                    root = tree.getroot()   
                    # A NAMESPACE miatt szükség lehet egy üres namespace kezelésére
                    namespaces = {'xacro': 'http://ros.org/wiki/xacro',
                    'dl': 'urn:ietf:params:xml:ns:yang:device-layer'
                    }
                    
                    device_name_element = root.find('.//dl:device-name', namespaces)
                    self.type = device_name_element.text.split('-')[0]

                    cat6_count = len(root.findall('.//xacro:cat6', namespaces))
                    cat5e_count = len(root.findall('.//xacro:cat5e', namespaces))
                    usb30_count = len(root.findall('.//xacro:usb30', namespaces))
                    usb20_count = len(root.findall('.//xacro:usb20', namespaces))
                    usbC_count = len(root.findall('.//xacro:usb-typec', namespaces))
                    portnumbers = {
                        'cat6': cat6_count,
                        'cat5e': cat5e_count,
                        'usb30': usb30_count,
                        'usb20': usb20_count,
                        'usb-typec': usbC_count
                    }

                    for port_type, count in portnumbers.items():
                        for i in range(count):
                            port = {
                                'type': port_type,
                                'id': f'{port_type}-{i}',
                                'free': 'yes'
                            }
                            self.ports.append(port)
         return
    
    def create_marker(self):
        global server, dynamic_markers, marker_pos, dynamic_marker_counter_id, node
        #interactive marker part
        self.int_marker = InteractiveMarker()
        dynamic_marker_counter_id = dynamic_marker_counter_id + 1
        self.int_marker.name = f"marker_{dynamic_marker_counter_id}"
        self.id = dynamic_marker_counter_id
        self.int_marker.header.frame_id = 'world' #int_marker.name 
        self.int_marker.pose.position.y = float(-3 * marker_pos)
        marker_pos +=1
        self.scale = 1.0
        
        #interactive marker control part
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        #marker part
        self.marker = Marker()
        self.marker.type = Marker.CUBE
        self.marker.scale.x = self.int_marker.scale * 0.45
        self.marker.scale.y = self.int_marker.scale * 0.45
        self.marker.scale.z = self.int_marker.scale * 0.45
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        #making the connections
        control.markers.append(self.marker)
        self.int_marker.controls.append(control)

        #new control mode with the same object
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        self.int_marker.controls.append(control)

        server.insert(self.int_marker, feedback_callback=lambda feedback, node=node: process_feedback(feedback, node.get_logger(), node))
        dynamic_markers.append((self.marker, self.int_marker))
        self.menu_handler.apply(server, self.int_marker.name)
        server.applyChanges()

#class ends here

        
def initMenu(objectFiles):
    global h_first_entry, h_mode_last, h_deploy_entry, h_device_entry, h_load_entry, h_save_entry, h_remove_entry, h_export_entry, h_sensor_entry, h_connect_entry, myPath
    
    patternforur = re.compile(r"^ur.{0,2}\.dae$")
    patternforgripper = re.compile(r".*_gripper\.(dae|stl)$")
    patternforabb = re.compile(r"^irb.*\.dae$")

    h_deploy_entry = menu_handler.insert('Deploy device', callback=deployDevice)
    h_device_entry = menu_handler.insert('Change device')

    h_sub_robot_entry = menu_handler.insert('Robot components', parent = h_device_entry)
    h_sub_network_entry = menu_handler.insert('Network components', parent = h_device_entry)
    h_rg2ft_entry = menu_handler.insert('Grippers', parent = h_sub_robot_entry)
    h_sensor_entry = menu_handler.insert('Sensors', parent= h_sub_robot_entry)
    h_realsense2_entry = menu_handler.insert('Onrobot cameras', parent=h_sub_robot_entry)
    h_holder_entry = menu_handler.insert('Camera holders', parent = h_sub_robot_entry)
    h_universal_robots_entry = menu_handler.insert('ur', parent=h_sub_robot_entry)
    h_abb_robots_entry = menu_handler.insert('abb', parent = h_sub_robot_entry)
    h_drone_entry = menu_handler.insert('drone', parent=h_sub_robot_entry)
    w = os.walk(myPath)

    for dirpath, dirnames, filenames in w:
        for object in objectFiles:
             for file in filenames:
                filepath = os.path.join(dirpath, file)
                if object in filepath:
                   # print (filepath)
                    if 'network-components' in filepath:
                        #print('Menu ' + filepath )
                        h_sub_device_entry = menu_handler.insert(object, parent = h_sub_network_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))
                    
                    if 'example_system_description' in filepath:
                            h_sub_device_entry = menu_handler.insert(object, parent = h_holder_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))
                    
                    if patternforgripper.match(file):
                            h_sub_device_entry = menu_handler.insert(object, parent = h_rg2ft_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))   
                    
                    if 'LD19.stl' in filepath or 'kwr75.dae' in filepath:
                              h_sub_device_entry = menu_handler.insert(object, parent = h_sensor_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))   
                    
                    if 'realsense2_description' in filepath:
                            h_sub_device_entry = menu_handler.insert(object, parent=h_realsense2_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))       
                    
                    if 'quadrotor' in filepath:
                        h_sub_device_entry = menu_handler.insert(object, parent=h_drone_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))
                    
                    if patternforabb.match(file):
                            #print('abb: ' + filepath)
                            h_sub_device_entry = menu_handler.insert(object, parent = h_abb_robots_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))
                        
                    if patternforur.match(file):
                            h_sub_device_entry = menu_handler.insert(object, parent = h_universal_robots_entry, callback=lambda feedback, path=filepath: changeDevice(feedback, path))
    
    h_load_entry = menu_handler.insert('Load', callback=load_markers_from_file)
    h_save_entry = menu_handler.insert('Save', callback=save_markers_to_file)
    h_remove_entry = menu_handler.insert('Remove', callback=removeDevice)
    h_export_entry = menu_handler.insert('Export', callback = exportMarkers) #markerExport

    #valahogyan ki kell deriteni, hogy melyik markerrol van szo, lehet mindegyik markernek sajat menuhandlerje?
    #egeszen ugy nez ki, hogy igen!
    h_connect_entry = menu_handler.insert('Connectivity')
    create_entry = menu_handler.insert('Create Connection', parent=h_connect_entry)
    
    #ide kell jonnie annak, hogy milyen szabad portjai vannak.
    destroy_entry = menu_handler.insert('Destroy Connection', parent=h_connect_entry)
    #ide pedig annak, hogy milyen elfoglalt portjai vannak.

    h_first_entry = menu_handler.insert('First Entry')
    entry = menu_handler.insert('deep', parent = h_first_entry)
    entry = menu_handler.insert('sub', parent = entry)
    entry = menu_handler.insert('menu', parent = entry)

    menu_handler.setCheckState(
        menu_handler.insert('Show First Entry', callback = enableCb),
        MenuHandler.CHECKED
    )

    sub_menu_handle = menu_handler.insert('Switch')

    for i in range(5):
        s = f"Mode{i}"
        h_mode_last = menu_handler.insert(s, parent = sub_menu_handle, callback = modeCb)
        menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)

    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)
    menu_handler.reApply(server)
    server.applyChanges()

def GetTheUrdfData(markermeshfile, i, x, y, z):
    urdfattachment = str()

    if (markermeshfile == 'intel_nuc.stl'):
        urdfattachment = f'<xacro:include filename="$(find nuc_description)/urdf/nuc.urdf.xacro" /> \n <xacro:nuc parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '
    
    if(markermeshfile == 'Mikrotik_hAPac2_Bottom.stl'):
        urdfattachment = f'<xacro:include filename="$(find mikrotik-description)/urdf/mikrotik.urdf.xacro" /> \n <xacro:mikrotik parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '
    #if(markermeshfile == 'Mikrotik_hAPac2_Top.stl'): 

    if(markermeshfile == 'radio_dot.stl'):
        urdfattachment = f' <xacro:include filename="$(find radiodot_description)/urdf/radiodot.urdf.xacro" /> \n <xacro:radiodot parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '

    if(markermeshfile == 'raspberry_pi_4_model_b.stl'):
        urdfattachment = f'<xacro:include filename="$(find raspi4b_description)/urdf/rpi4.urdf.xacro" /> \n <xacro:rpi4 parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '
    
    if(markermeshfile == 'server-rack.stl'):
        urdfattachment = f'<xacro:include filename="$(find flightrack_description)/urdf/flightrack.urdf.xacro" /> \n <xacro:flightrack parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '

    if(markermeshfile == 'TP-LINK_5_Port_POE_Switch_Blank_TL-SG1005P.stl'):
        urdfattachment = f'<xacro:include filename="$(find poesw_description)/urdf/poe-switch.urdf.xacro" /> \n <xacro:poesw parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '

    if(markermeshfile == 'usb-dongle.stl'):
        urdfattachment = f'<xacro:include filename="$(find nw-5g-modem_description)/urdf/nw-5g-modem.urdf.xacro" /> \n <xacro:nw-5g-modem parent="world" id="{i}" xyz="{x} {y} {z}"/>\n '
    #itt kell kiboviteni a tobbi cumoval, esetleg atirni az egeszet
    #robotok
    if(markermeshfile == 'irb1200_5_90.dae'):
        urdfattachment=f'<xacro:include filename="$(find abb_irb1200_support)/urdf/irb1200_5_90_macro.xacro" />\n<xacro:abb_irb1200_5_90 parent = "world" id = "{i}" xyz= "{x} {y} {z}"/>\n'
    if(markermeshfile == 'irb4600_20_250.dae'):
        urdfattachment=f'<xacro:include filename="$(find abb_irb4600_support)/urdf/irb4600_20_250_macro.xacro" />\n<xacro:abb_irb4600_20_250 parent = "world" id = "{i}" xyz= "{x} {y} {z}"/>\n'
    if(markermeshfile == 'irb4600_40_255.dae'):
        urdfattachment=f'<xacro:include filename="$(find abb_irb4600_support)/urdf/irb4600_40_255_macro.xacro" />\n<xacro:abb_irb4600_40_255 parent = "world" id = "{i}" xyz="{x} {y} {z}"/>\n'
    if(markermeshfile == 'irb4600_60_205.dae'):
        urdfattachment=f'<xacro:include filename="$(find abb_irb4600_support)/urdf/irb4600_60_205_macro.xacro" />\n<xacro:abb_irb4600_60_205 parent = "world" id = "{i}" xyz="{x} {y} {z}"/>\n'
    #szenzorok
    if(markermeshfile == 'kwr75.dae'):
        urdfattachment=f'<xacro:include filename="$(find kwr75_force_sensor_ros2_nw_arch)/urdf/kwr75_macro.urdf.xacro" />\n<xacro:kwr75   parent="world" id="{i}" xyz="{x} {y} {z}"/>\n'
    if(markermeshfile == 'LD19.stl'):
        urdfattachment=f'<xacro:include filename="$(find ldrobot-lidar-ros2_nw_arch)/ldlidar_node/urdf/ldlidar_descr.urdf.xacro" />\n<xacro:ldlidar parent="world" id="{i}" xyz="{x} {y} {z}"/>'
    #gripper
    if(markermeshfile == 'rg2ft.stl'):
        urdfattachment=f'<xacro:include filename="$(find onrobot_rg2ft_description)/urdf/onrobot_rg2ft_macro.xacro" />\n <xacro:onrobot_rg2ft prefix="{i}" hw_interface="hardware_interface/PositionJointInterface" parent="world"> <origin xyz="{x} {y} {z}" rpy="0 0 1.5707963" /> </xacro:onrobot_rg2ft>\n'
    if(markermeshfile == '2f_85.dae'):
        urdfattachment=f'<xacro:include filename="$(find robotiq_descripiton)/urdf/robotiq_2f_85_macro.urdf.xacro" />\n <xacro:robotiq_gripper name="name" prefix={i} parent="world" <origin xyz="{x} {y} {z}" rpy="0 1.57 0" /> </xacro:robotiq_gripper>\n'
    #drone
    if(markermeshfile == 'quadrotor_4.dae'):
        urdfattachment = f'<xacro:include filename="$(find sjtu_drone_descrpition)/urdf/sjtu_drone.urdf.xacro" />\n<xacro:sjtu-drone parent="world" id="{i}" xyz="{x} {y} {z}"/>\n'
    #cameras
    if(markermeshfile == 'd415.stl'):
        urdfattachment=f'<xacro:include filename="$(find realsense2_description)/urdf/_d415.urdf.xacro" />\n <xacro:sensor_d415 parent="world">\n  <origin xyz="{x} {y} {z}" rpy="0 1.57 0" />\n </xacro:sensor_d415>\n'
    if(markermeshfile == 'd435.dae'):
        urdfattachment=f'<xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro" />\n <xacro:sensor_d435 parent="world">\n  <origin xyz="{x} {y} {z}" rpy="0 1.57 0" />\n </xacro:sensor_d435>\n'
    if(markermeshfile == 'd455.stl'):
        urdfattachment=f'<xacro:include filename="$(find realsense2_description)/urdf/_d455.urdf.xacro" />\n <xacro:sensor_d455 parent="world">\n  <origin xyz="{x} {y} {z}" rpy="0 1.57 0" />\n </xacro:sensor_d455>\n'
    if(markermeshfile == 'l515.dae'):
        urdfattachment=f''
    #camera holders
    if(markermeshfile == 'gripper_bracket_with_screws.stl'):
        urdfattachment=f'<xacro:include filename="$(find example_system_description)/urdf/gripper_bracket.urdf.xacro" />\n <xacro:gripper_bracket parent="world" id="{i}" xyz="{x} {y} {z}" />'
    if(markermeshfile == 'skamera_ur_new.stl'):
        urdfattachment=f'<xacro:include filename="$(find example_system_description)/urdf/camera_holder.urdf.xacro" />\n <xacro:camera_holder parent="world" xyz="{x} {y} {z}" />'
    return urdfattachment         

def exportMarkers(feedback):
    global dynamic_markers, objectFiles, robotic_arms
    #path = "./src/urdf_yang_extension/example_system_description/urdf/robot_system_description.urdf.xacro"
    path = "./src/urdf_yang_extension/example_system_description/urdf/test.urdf.xacro"
    lines = []

    with open(path, 'r') as file:
        for line in file:
            lines.append(line)
            if "<!--imported part-->" in line:
                break
        lines.append('</robot>')
    
    new_xacro_content = []
    
    for marker,int_marker in dynamic_markers:
            markermeshfile = marker.mesh_resource.replace("file://", "")
            markermeshfile = os.path.basename(markermeshfile)
            if 'ur' not in markermeshfile: #az export itt tortenne meg
                i = int_marker.name.split("_")[1]
                new_xacro_content.append(GetTheUrdfData(markermeshfile,i, int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z))
    for arm in robotic_arms:
        for marker, int_marker in dynamic_markers:
            if arm.markername == int_marker.name:
                new_xacro_content.append(arm.export(int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z))
    #new_xacro_content.append(all_robot_material)      
    for i,line in enumerate(lines):
        if line.strip() == '</robot>':
            lines = lines[:i] + new_xacro_content + lines[i:]
            break
    
    with open(path, 'w') as file:
        file.writelines(lines)

def load_markers_from_file(feedback):
    global dynamic_marker_counter_id, dynamic_markers, menu_handler, node
    markers = []
    filename = "./src/urdf_yang_extension/marker_extension/marker_extension/creativenameneeded.json"

    with open(filename, 'r') as file:
           
            data = json.load(file)
           # print(f"Loaded {len(data)} markers from file.")

            for marker_data in data:
                marker = Marker()
                marker.type = marker_data['type']
                marker.scale.x = marker_data['scale'][0] 
                marker.scale.y = marker_data['scale'][1] 
                marker.scale.z = marker_data['scale'][2]
                marker.color.r = marker_data['color']['r']
                marker.color.g = marker_data['color']['g']
                marker.color.b = marker_data['color']['b']
                marker.color.a = marker_data['color']['a']
                pose = Pose()
                pose.position.x = marker_data['pose']['position']['x']
                pose.position.y = marker_data['pose']['position']['y']
                pose.position.z = marker_data['pose']['position']['z']
                pose.orientation.x = marker_data['pose']['orientation']['x']
                pose.orientation.y = marker_data['pose']['orientation']['y']
                pose.orientation.z = marker_data['pose']['orientation']['z']
                pose.orientation.w = marker_data['pose']['orientation']['w']
                marker.mesh_resource = marker_data['mesh_resource'] 

                markers.append(marker)

                dynamic_marker_counter_id = dynamic_marker_counter_id + 1
                int_marker = InteractiveMarker()
                int_marker.header.frame_id = marker_data['header']['frame_id']
                int_marker.name = f"marker_{dynamic_marker_counter_id}"
                int_marker.pose = pose
                control = InteractiveMarkerControl()
                control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
                control.always_visible = True
                control.markers.append(marker)
                int_marker.controls.append(control)

                #print(f"Interactive marker {int_marker.name} created")
# 
                dynamic_markers.append((marker, int_marker))
                server.insert(int_marker, feedback_callback=lambda feedback, node=node: process_feedback(feedback, node.get_logger(), node))

                menu_handler.apply(server, int_marker.name)
              
    server.applyChanges()
   # print("Markers loaded successfully!")      

def save_markers_to_file(feedback):
    global dynamic_markers
    filename = "./src/urdf_yang_extension/marker_extension/marker_extension/creativenameneeded.json"
    data = []

    node.get_logger().info('save_markers_to_file()')
  #  print("markers array length:" + str(len(dynamic_markers)))

    for (marker, int_marker) in dynamic_markers:

        marker_data = {
            'header':{
                'frame_id' : int_marker.header.frame_id
            },
            'type': marker.type,
            'scale': [marker.scale.x,marker.scale.y,marker.scale.z],
            'color': {
                'r': marker.color.r,
                'g': marker.color.g,
                'b': marker.color.b,
                'a': marker.color.a,
            },
            'pose': {
                'position':{
                    'x' : int_marker.pose.position.x,
                    'y' : int_marker.pose.position.y,
                    'z' : int_marker.pose.position.z,
                },
                'orientation':{
                    'x': int_marker.pose.orientation.x,
                    'y': int_marker.pose.orientation.y,
                    'z': int_marker.pose.orientation.z,
                    'w': int_marker.pose.orientation.w,
                }
            },
            'mesh_resource': marker.mesh_resource,
        }

        data.append(marker_data)
    
    with open(filename, 'w') as file:
        json.dump(data, file, indent=2)

#Menu ends here

def process_feedback(feedback, logger, node):
    logger.info(f"{feedback.marker_name} is now at {feedback.pose.position.x}, " f"{feedback.pose.position.y}, {feedback.pose.position.z}")
    
    transformation = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z,
                      feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w]
    
    node.update_transform(feedback.marker_name, transformation)

class FramePublisher(Node):
    """
    Publishes transforms from `base_link` to a static frame.
    """

    def __init__(self): #transformations
        super().__init__('tf2_frame_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.transform_dict = {}
        self.timer = self.create_timer(0.1, self.publish_transformations)  # Create timer to periodically publish transforms
        

    def publish_transformations(self):
        for key in self.transform_dict:
            self.make_transforms(self.transform_dict[key], key)

    def make_transforms(self, transformation, key):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = str(key)

        t.transform.translation.x = float(transformation[0])
        t.transform.translation.y = float(transformation[1])
        t.transform.translation.z = float(transformation[2])
        t.transform.rotation.x = float(transformation[3])
        t.transform.rotation.y = float(transformation[4])
        t.transform.rotation.z = float(transformation[5])
        t.transform.rotation.w = float(transformation[6])

        self.tf_broadcaster.sendTransform(t)

    def update_transform(self, marker_name, transformation):
        self.transform_dict[marker_name] = transformation
    
   

def main(args=None):
    global server, node
    # pass parameters and initialize node
    rclpy.init(args=sys.argv)
    
    #node = rclpy.create_node('menu')
    node = FramePublisher()
    # create an interactive marker server on the namespace "simple_marker"
    server = InteractiveMarkerServer(node, "menu")

    objectFiles = getObjectList(myPath)
    initMenu(objectFiles)
    #makeMenuMarker()
    markerwithmenu = MarkerWithMenu()

    # 'commit' changes and send to all clients
    server.applyChanges()
    node.get_logger().info("Ready")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()