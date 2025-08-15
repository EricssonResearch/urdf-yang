import rclpy
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
import tf2_ros
import tf2_geometry_msgs
import tf2_py
import threading
import time
import yaml
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import InteractiveMarkerControl
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarkerFeedback


#http://docs.ros.org/en/jade/api/interactive_markers/html/interactive__marker__server_8py_source.html

DEBUG = 0
node = None
buffer = None
interactive_server = None

all_frames = []
static_markers = []
static_marker_counter_id = 0
dynamic_markers = []

active_interfaces = dict()
channels = dict()
channel_pairs = []
name1 = []
name2 = []

#TF part
class TFListenerThread(threading.Thread):
   
    def __init__(self, buffer, node):
        super().__init__()
        self.buffer = buffer
        self.node = node

    def run(self):

        def timer_callback():
            global all_frames
            try:
                # List of frames available in RViz2
                frames_dict = yaml.safe_load(self.buffer.all_frames_as_yaml())
                all_frames = list(frames_dict.keys())
                self.node.get_logger().info("Available TF Frames: {}".format(all_frames)) if DEBUG == 1 else 0

            except Exception as e:
                self.node.get_logger().error("Transform lookup failed: {}".format(str(e)))

        # Create a timer to collect data for 1 seconds
        timer_period = 0.1  # 1 second
        timer = self.node.create_timer(timer_period, timer_callback)

def processFeedback(feedback):
    # Handle user interactions here
    p = feedback.pose.position
    print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')

#TF part end

#Channel part
def alignMarker(feedback):
    global node, interactive_server, static_markers, channel_pairs, name1, name2
   
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    print("Publishing static links") if DEBUG == 1 else 0
    
    for link_name, marker in static_markers:
        link_name_words = []
        link_name_words = link_name.split('_')

        if len(link_name_words)>3:
            
            transform = buffer.lookup_transform('world', link_name, rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1.0))
            point_wrt_source = Point()
            point_wrt_source.x = marker.pose.position.x
            point_wrt_source.y = marker.pose.position.y
            point_wrt_source.z = marker.pose.position.z
            point = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source), transform).point

            point_wrt_source2 = Point()
            point_wrt_source2.x = feedback.pose.position.x
            point_wrt_source2.y = feedback.pose.position.y
            point_wrt_source2.z = feedback.pose.position.z
            transform2 = buffer.lookup_transform('world', 'world', rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1.0))
            point2 = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source2), transform2).point

            dist = ((point.x - point2.x)**2 + (point.y - point2.y)**2 + (point.z - point2.z)**2)**(1/2)

            print("\tlink name: " + str(link_name) + " dist "  + str(dist)) if DEBUG == 1 else 0

            if dist < 0.1 and len(channel_pairs) ==0 and active_interfaces.get(marker.header.frame_id)==None:
                print("\tdist ok: " + str(link_name)) #if DEBUG == 1 else 0 
                channel_pairs.append(marker)
                name1 = link_name_words
                pose = Pose()
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                interactive_server.setPose(feedback.marker_name, pose)
                interactive_server.applyChanges()
            #ha nem tetszik, akkor a zold gombhoz oda lehet vinni, ami lepucolja az egeszet, es lehet elorol kezdeni.


            if dist < 0.1 and len(channel_pairs) == 1 and channel_pairs[0] != marker and len(name1)>0 and active_interfaces.get(marker.header.frame_id)==None:
                name2 = link_name_words
                if (name1[2] == name2[2]) and ((name1[0] + name1[1]) != (name2[0] + name2[1])): #or name1[1] == name2[2] or name1[2] == name2[1] #ez a feltetel lehet nem is kell
                    print("\tdist ok: " + str(link_name))
                    channel_pairs.append(marker)
                    pose = Pose()
                    pose.position.x = 0.0
                    pose.position.y = 0.0
                    pose.position.z = 0.0
                    interactive_server.setPose(feedback.marker_name, pose)
                    interactive_server.applyChanges()
            #ha a ket port tipusa nem egyezik meg, akkor pucolas van
            elif dist < 0.1 and len(channel_pairs) == 1:
                name2 = link_name_words
                if (name1[2] != name2[2]) or ((name1[0] + name1[1]) == (name2[0] + name2[1])):
                    #channel_pairs.clear()
                    #name1 = []
                    print("Nem jo!")
                    name2 = []
                    
                
            if len(channel_pairs) == 2 and name1 != name2 and (name1[2] == name2[2]): #or name1[1] == name2[2] or name1[2] == name2[1]
                #print("mind_dist_marker: " + str(min_dist_marker.header.frame_id))
                create_channel(channel_pairs[0], channel_pairs[1])
                print(f"Channel created called with {channel_pairs[0].header.frame_id} and {channel_pairs[1].header.frame_id}")
                channel_pairs.clear()
                name1 = []
                name2 = []
                break
            else:
                if len(channel_pairs) ==2:
                    channel_pairs.clear()
            
        interactive_server.applyChanges()

#nw-5g-modem_1_usb-a-30_0_link and nuc_5_usb-a-30_0_link
def create_channel(marker_from, marker_to):
    global static_markers, static_marker_counter_id, channels, buffer, active_interfaces

    if marker_from != marker_to:
        link_from_name = str()
        link_to_name = str()

        for link_name, marker in static_markers:
            if marker_from == marker:
                print("Egyet talaltam!" + str(link_name)) if DEBUG == 1 else 0
                link_from_name = link_name

        for link_name, marker in static_markers:
            if marker_to == marker:
                print("Kettot talaltam!" + str(link_name)) if DEBUG ==1 else 0
                link_to_name = link_name

        words_from = []
        words_to = []

        if link_from_name is not None and link_to_name is not None:
            words_from = link_from_name.split("_")
            words_to = link_to_name.split("_")
        
        if (len(words_from)>3 and len(words_to)>3) and (words_from[-1] == 'link') and (words_to[-1] == 'link') and (words_to[2] == words_from[2]): #or words_to[1] == words_from[2] or words_to[2] == words_from[1]

                one_direction = "channel:" + marker_from.header.frame_id + ":" + marker_to.header.frame_id
                rev_direction = "channel:" + marker_to.header.frame_id + ":" + marker_from.header.frame_id

                if channels.get(one_direction) == None and channels.get(rev_direction) == None and active_interfaces.get(marker_from.header.frame_id) == None and active_interfaces.get(marker_to.header.frame_id) == None:
                    print("create_channel: " + marker_from.header.frame_id + ":" + marker_to.header.frame_id)
                    marker = Marker()
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.id = static_marker_counter_id
                    # https://answers.ros.org/question/228238/reasons-for-marker-or-markerarray-to-be-invisible-in-rviz/
                    # ennek rendes publish-olt tf-nek kell lennie
                    marker.header.frame_id = marker_from.header.frame_id
                    marker.pose.position.x = 0.0  # Set the desired position
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1
                    marker.color.r = 0.0
                    marker.color.g = 0.0 
                    marker.color.b = 1.0
                    marker.color.a = 1.0

                    #ezek mindig 0-ak lesznek, hiszen csak a tf-hez kepest vannak elterve
                    # ezek 0,0,0-ak legyenek mindig
                    pose_from = Point() #Pose()
                    pose_from.x = 0.0
                    pose_from.y = 0.0
                    pose_from.z = 0.0
                    marker.points.append(pose_from)

                    #to-koordinata rendeszert atrakom a from koordinata rendszerebe
                    transform = buffer.lookup_transform(marker_from.header.frame_id,marker_to.header.frame_id,  rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1.0))
                    # Use the dynamic TF link's position in 'transform' to update the marker's position
                    point_wrt_source = Point()
                    point_wrt_source.x = 0.0
                    point_wrt_source.y = 0.0
                    point_wrt_source.z = 0.0
                    
                    pose_to = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source), transform).point
                    print("\t\tmarker pos: " + str(pose_to.position)) if DEBUG == 1 else 0

                    marker.points.append(pose_to)
                    static_markers.append((marker.header.frame_id, marker))
                    static_marker_counter_id = static_marker_counter_id + 1
                    print("static_marker_counter_id: " + str(static_marker_counter_id))

                    channels[one_direction] = 1
                    channels[rev_direction] = 1
                    active_interfaces[marker_from.header.frame_id] = 1
                    active_interfaces[marker_to.header.frame_id] = 1 

                    interactive_server.applyChanges()

def export_channel(filename):
    global  channels
    list_of_channels = list(channels.keys())

    try:
            with open(filename, 'w') as file:
                file.write("")

            for i, channel in enumerate(list_of_channels):
                if i % 2 ==1:
                    continue
                channel_parts = []
                channel_parts = str(channel).split(":")
                from_parts = []
                from_parts = channel_parts[1].split("_")
                to_parts = []
                to_parts = channel_parts[2].split("_")
                if 'wireless' in from_parts:
                    conn_type = 'wireless'
                else :
                    conn_type = 'wired'
                with open(filename, 'a') as file:
                    file.write(f'<xacro:topology-link src_node = "{from_parts[0]}_{from_parts[1]}" src_tp="{from_parts[2]}_{from_parts[3]}" dst_node="{to_parts[0]}_{to_parts[1]}" dst_tp="{to_parts[2]}_{to_parts[3]}" conn_type="{conn_type}" reconfig="sw_supported"/>\n')
    
    except() as e:
        node.get_logger().error("Open failed: {}".format(str(e)))     

#Channel part end

#Link and marker part 
def get_static_tf_links():
    global all_frames, buffer
    """
    Get the names of all static TF links in the TF tree.
    """
    print("get_static_tf_links: " + str(all_frames)) if DEBUG == 1 else 0
    static_links = []
    dynamic_links = []
    for frame in all_frames:
        print("frame: " + str(frame)) if DEBUG == 1 else 0
        transform = buffer.lookup_transform('world', frame, rclpy.time.Time().to_msg())
        print("\ttransform time: " + str(transform.header.stamp)) if DEBUG == 1 else 0
       
        if transform.header.stamp.sec == 0:
            print("\tStatic transformation found.") if DEBUG == 1 else 0
            static_links.append(frame)
        else:
            print("\tReal-time transformation found.") if DEBUG == 1 else 0
            dynamic_links.append(frame)

    return static_links, dynamic_links

def get_dynamic_tf_links(static_links):
    global all_frames, buffer
    """
    Get the names of all dynamic TF links in the TF tree.
    """
    print("get_dynamic_tf_links: " + str(all_frames)) if DEBUG == 1 else 0
    dynamic_links = [frame for frame in all_frames if frame not in static_links]
    return dynamic_links

def create_static_markers(static_links):
    global static_marker_counter_id, buffer
    print("create_static_markers") if DEBUG == 1 else 0
    # Create static markers for all static TF links
    static_markers = []

    static_marker_counter_id = 0
    for link_name in static_links:

        print("Link name: " + str(link_name)) if DEBUG == 1 else 0
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = static_marker_counter_id
        marker.header.frame_id = link_name

        marker.pose.position.x = 0.0  # Set the desired position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.color.r = 0.0
        marker.color.g = 1.0  # Change the color as needed
        marker.color.b = 0.0
        marker.color.a = 1.0
    
        static_markers.append((link_name, marker))
        static_marker_counter_id = static_marker_counter_id + 1
        # lehetne meg feliratot is csinalni hozzajuk:
        # View-Oriented Text (TEXT_VIEW_FACING=9) [1.1+]
        # http://wiki.ros.org/rviz/DisplayTypes/Marker

    return static_markers

def create_dynamic_markers(dynamic_links, interactive_server):
    global buffer
    print("crete_dynamic_markers") if DEBUG == 1 else 0
    # Create dynamic markers for interactive markers
    dynamic_markers = []

    for link_name in dynamic_links: 
        print("Link name: " + str(link_name)) if DEBUG == 1 else 0

        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = link_name
        interactive_marker.name = link_name
        interactive_marker.description = link_name
        interactive_marker.pose.position.x = 0.0
        interactive_marker.pose.position.y = 0.0
        interactive_marker.pose.position.z = 0.0
        interactive_marker.scale = 0.5

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0  # Set the desired position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0  # Change the color as needed
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.header.frame_id = link_name

        # create a non-interactive control which contains the box
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.name = link_name
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(marker)

        interactive_marker.controls.append(sphere_control)
        interactive_server.insert(interactive_marker, feedback_callback=processFeedback)

        # set different callback for POSE_UPDATE feedback
        interactive_server.setCallback(interactive_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE)

        dynamic_markers.append((link_name, interactive_marker))

    for i in range(2):
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "world"
        interactive_marker.name = f"base_link_{i}" 
        interactive_marker.description = "base_link"
        interactive_marker.pose.position.x = 0.0
        interactive_marker.pose.position.y = 0.0
        interactive_marker.pose.position.z = 0.0
        interactive_marker.scale = 0.5

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.pose.position.x = 0.0  # Set the desired position
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0  # Change the color as needed
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        #ettol jo helyre ugranak az interactive markerek, viszont a pose.update utani callback nem nagyon akar lefutni

        # create a non-interactive control which contains the box
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.name = link_name
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(marker)

        interactive_marker.controls.append(sphere_control)
        interactive_server.insert(interactive_marker, feedback_callback=processFeedback)

        # set different callback for POSE_UPDATE feedback
        interactive_server.setCallback(interactive_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE)

        dynamic_markers.append(("world", interactive_marker))
    
    return dynamic_markers


def publish_static_markers(marker_publisher, static_markers):
    global buffer
    print("Publishing static markers") if DEBUG == 1 else 0
    for link_name, marker in static_markers:
        print("\tlink name: " + str(link_name)) if DEBUG == 1 else 0
        # Modify marker position, orientation, or other properties as needed
        #if link_name.find('-') == -1:
         #   try:
                #ugy tunik semmi szukseg transformationre, arra a tf link-re rakja ami a header
           #     1
         #   except (tf2_py.LookupException, tf2_py.ConnectivityException, tf2_py.ExtrapolationException) as e:
                # Handle TF lookup exceptions
           #     node.get_logger().error("Transform lookup failed: {}".format(str(e)))
           #     pass
       # else: 
         #   print("\t\tPublish channel") if DEBUG == 1 else 0
        
        marker_publisher.publish(marker)

def publish_dynamic_markers(marker_publisher, dynamic_markers, interactive_server):
    global buffer
    print("Publishing dynamic markers") if DEBUG == 1 else 0
    for link_name, interactive_marker in dynamic_markers:
        print("\tlink name: " + str(link_name)) if DEBUG == 1 else 0
        try:
            transform = buffer.lookup_transform("world", link_name, rclpy.time.Time().to_msg(), rclpy.duration.Duration(seconds=1.0))
            # Use the dynamic TF link's position in 'transform' to update the marker's position
            point_wrt_source = Point()
            point_wrt_source.x = 0.0
            point_wrt_source.y = 0.0 #1
            point_wrt_source.z = 0.0 #2

            interactive_marker.pose.position = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source), transform).point
            print("\t\tmarker pos: " + str(interactive_marker.pose.position)) if DEBUG == 1 else 0

        except (tf2_py.LookupException, tf2_py.ConnectivityException, tf2_py.ExtrapolationException) as e:
            # Handle TF lookup exceptions
            node.get_logger().error("Transform lookup failed: {}".format(str(e)))
            pass

    interactive_server.applyChanges()

#Link and marker part end

def main(args=None):
    global node, interactive_server, static_markers, dynamic_markers, buffer
    rclpy.init()
    node = rclpy.create_node('marker_node')

    buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
    listener = tf2_ros.TransformListener(buffer, node)
    tf_listener_thread = TFListenerThread(buffer, node)
    tf_listener_thread.start()

    frames_dict = yaml.safe_load(buffer.all_frames_as_yaml())
    print("frames_dict: " + str(frames_dict)) if DEBUG == 1 else 0

    marker_publisher = node.create_publisher(Marker, 'your_marker_topic',150) #ezzel a szammal kell mokolni, hogy milyen gyorsan jelenjen meg a channel marker

    interactive_server = InteractiveMarkerServer(node, 'your_interactive_marker_server')

    static_link_prev = []
    dynamic_links_prev = []
    try:
        while rclpy.ok():
            print("rclpy.ok()") if DEBUG == 1 else 0
            print("Collecting links") if DEBUG == 1 else 0
            static_links, dynamic_links = get_static_tf_links()
            if static_links != static_link_prev:
                static_markers = create_static_markers(static_links)

            #print("Collecting dynamic links")
            if dynamic_links != dynamic_links_prev:
                dynamic_markers = create_dynamic_markers(dynamic_links, interactive_server)

            publish_static_markers(marker_publisher, static_markers)
            publish_dynamic_markers(marker_publisher, dynamic_markers, interactive_server)

            static_link_prev = static_links
            dynamic_links_prev = dynamic_links
            interactive_server.applyChanges()
            
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        filename = 'channels.txt'
        print('channel meghivva')
        export_channel(filename)
    
    node.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()
