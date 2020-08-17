import bagpy
import rosbag
import numpy as np
from numpy import save


bag=rosbag.Bag("D:/ford.bag")



def process_bag(path):
    bag = rosbag.Bag(path)
    vel = []
    """
    topics:      /gps                 170441 msgs @ 155.0 Hz : sensor_msgs/NavSatFix       
                 /gps_time            170441 msgs @ 155.0 Hz : sensor_msgs/TimeReference   
                 /image_front_left     11777 msgs @  14.7 Hz : sensor_msgs/Image           
                 /imu                 170441 msgs @ 155.0 Hz : sensor_msgs/Imu             
                 /lidar_blue_scan       8497 msgs @   9.3 Hz : velodyne_msgs/VelodyneScan  
                 /lidar_green_scan      8503 msgs @   9.3 Hz : velodyne_msgs/VelodyneScan  
                 /lidar_red_scan        8506 msgs @   9.3 Hz : velodyne_msgs/VelodyneScan  
                 /lidar_yellow_scan     8492 msgs @   9.3 Hz : velodyne_msgs/VelodyneScan  
                 /pose_ground_truth   169125 msgs @ 199.8 Hz : geometry_msgs/PoseStamped   
                 /pose_localized       18328 msgs @  20.0 Hz : geometry_msgs/PoseStamped   
                 /pose_raw            169676 msgs @ 199.8 Hz : geometry_msgs/PoseStamped   
                 /tf                  169125 msgs @ 199.8 Hz : tf2_msgs/TFMessage          
                 /velocity_raw        169676 msgs @ 199.8 Hz : geometry_msgs/Vector3Stamped"""

    for topic, msg, t in bag.read_messages(topics=["/velocity_raw"]):
        vel.append(msg.vector.x)
    vel= np.array(vel)

    return np.save(vel)

