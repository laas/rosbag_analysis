#!/usr/bin/env python

# This script takes a rosbag as input and creates a directory
# containing a flag representation of the rosbag data which
# can be represented as curves (i.e. binary data and strings
# are ignored).
#
# The serialization is done by hand so only a few types are
# handled so far.

import roslib; roslib.load_manifest('rosbag_analysis')

import os
import os.path
import sys

import rospy
import rosbag

import tf.transformations

from geometry_msgs.msg import TransformStamped

if len(sys.argv) < 2:
    print("rosbag_analyzer.py ROSBAG")
    sys.exit(1)

bag_filename = sys.argv[1]

if not os.path.exists(bag_filename):
    print("bag file does not exist")
    sys.exit(2)

bag = rosbag.Bag(bag_filename)

print(bag)
print("Exporting data as text file, wait please...")

base_dir = os.path.splitext(bag_filename)[0]

if not os.path.exists(base_dir):
    os.makedirs(base_dir)

files = {}

def printDoubleArray(msg):
    res = ''
    for val in msg:
        res += '{0} '.format(val)
    return res

def printWrench(msg):
    return '{0} {1}'.format(printXYZ(msg.force), printXYZ(msg.torque))

def printXYZ(msg):
    return '{0} {1} {2}'.format(msg.x, msg.y, msg.z)

def printQuaternion(msg):
    q = [msg.x, msg.y, msg.z, msg.w]
    euler = tf.transformations.euler_from_quaternion(q)
    return '{0} {1} {2} {3} {4} {5} {6}'.format(
        msg.x, msg.y, msg.z, msg.w, euler[0], euler[1], euler[2])

def printTransform(msg):
    return '{0} {1}'.format(printXYZ(msg.translation),
                            printQuaternion(msg.rotation))

def printTimestamp(msg):
    return '{0} {1}'.format(msg.secs, msg.nsecs)

def printHeader(msg):
    return '{0} {1}'.format(msg.seq, printTimestamp(msg.stamp))

def write(topic, msg, t):
    global files
    if not topic in files:
        filename = base_dir + '/' + topic + '.dat'
        directory = os.path.split(filename)[0]

        if not os.path.exists(directory):
            os.makedirs(directory)
        files[topic] = open(filename, 'w')

    supportedMessageType = False

    if hasattr(msg, "header"):
        try:
            files[topic].write(printHeader(msg.header)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    if hasattr(msg, "transform"):
        try:
            files[topic].write(printTransform(msg.transform)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    if hasattr(msg, "vector"):
        try:
            files[topic].write(printXYZ(msg.vector)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    if hasattr(msg, "wrench"):
        try:
            files[topic].write(printWrench(msg.wrench)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    if hasattr(msg, "orientation"):
        try:
            files[topic].write(printQuaternion(msg.orientation)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass
    if hasattr(msg, "angular_velocity"):
        try:
            files[topic].write(printXYZ(msg.angular_velocity)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass
    if hasattr(msg, "linear_acceleration"):
        try:
            files[topic].write(printXYZ(msg.linear_acceleration)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    # Joint state.
    if hasattr(msg, "position"):
        try:
            files[topic].write(printDoubleArray(msg.position)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass
    if hasattr(msg, "velocity"):
        try:
            files[topic].write(printDoubleArray(msg.velocity)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass
    if hasattr(msg, "effort"):
        try:
            files[topic].write(printDoubleArray(msg.effort)+ ' ')
            supportedMessageType = True
        except AttributeError:
            pass

    # Add a newline if something has been printed.
    if supportedMessageType:
        files[topic].write('\n')

for topic, msg, t in bag.read_messages():
    write(topic, msg, t)

for k, v in files.iteritems():
    v.close()
bag.close()
