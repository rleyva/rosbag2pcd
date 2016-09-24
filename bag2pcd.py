import argparse
import sys
import rospy
import roslib
import rosbag
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def pc2_to_pcd(pc2_list, index):
    p = pcl.PointCloud()
    p.from_array(np.asarray(pc2_list, dtype='float32'))
    p.to_file(str(index) + ".pcd")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Convert select ROS PointCloud2 msg to PCD')
    parser.add_argument('-f', '--filepath', help='Input bag files')
    parser.add_argument('-o', '--output_dir', help='Output directory')
    parser.add_argument('-i', '--initial_cloud',
                        help='Message in bag containing desired cloud')
    parser.add_argument('-e', '--final cloud', help="Message index containing final cloud")

    args = parser.parse_args()

    # Input file source validation
    if args.filepath is not None:
        filepath = args.filepath
    else:
        print 'No source bag provided'
        filepath = sys.path[0]

    try:
        bag = rosbag.Bag(filepath)
    except:
        print 'Unhandled exception occured when opening file'

    cloud_topics, types, bag_msg_cnt = [], [], []
    topics = bag.get_type_and_topic_info()[1].keys()

    for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])
        bag_msg_cnt.append(bag.get_type_and_topic_info()[1].values()[i][1])

    topics = zip(topics, types, bag_msg_cnt)
    index = 0

    for topic, type, count in topics:
        if type == 'sensor_msgs/PointCloud2':
            print 'Topic(s) Found:'
            print '   ' + topic
            cloud_topics.append(topic)

    for topic, msg, t in bag.read_messages(topics=cloud_topics): 
        p_ = []
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))

        for p in gen:
            p_.append(p)

        pc2_to_pcd(p_, index)
        index = index + 1
