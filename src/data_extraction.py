import rospy
import numpy
import rosbag_pandas
import yaml

bn = '/home/waxz/bag/m.bag'
import rosbag

bag = rosbag.Bag(bn)

g = bag.read_messages(topics='/scan')
topic, msg, t = g.next()
print(msg)
f = open("data.yaml", "w")
dict = {}
dict["ranges"] = msg.ranges
yaml.dump(dict, f)
bag.close()
