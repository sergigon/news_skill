#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Sergio Gonzalez Diaz"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Sergio Gonzalez Diaz"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Sergio Gonzalez Diaz"
__email__ = "sergigon@ing.uc3m.es"
__status__ = "Development"

import rospy
from interaction_msgs.msg import CA
from common_msgs.msg import KeyValuePair as kvpa

def makeCA_etts_info(etts_text):
    """
    Makes a etts CA message.

    @param etts_text: Text to be sent.

    @return msg: CA message.
    """

    msg_name = str(rospy.get_rostime().nsecs)

    msg = CA()
    msg.type = "robot_giving_info"
    msg.ca_name = msg_name
    msg.duration = 0
    msg.priority = 1
    msg.emitter = "newspaper_ca"
    
    kvp = kvpa()
    kvp.key = "etts_text"
    kvp.value = etts_text
    msg.values.append(kvp)
    rospy.logdebug("Sending CA_info (%s)" % msg_name)
    return msg

def makeCA_tablet_info(image_url, image_type):
    """
    Makes a tablet CA message.

    @param image_url: Url of the image to be sent.
    @param image_type: Type of the image to be sent.

    @return msg: CA message.
    """

    msg_name = str(rospy.get_rostime().nsecs)

    msg = CA()
    msg.type = "robot_giving_info"
    msg.ca_name = msg_name
    msg.duration = 0
    msg.priority = 1
    msg.emitter = "newspaper_ca"
    
    kvp = kvpa()
    kvp.key = "tablet_type"
    kvp.value = image_type
    msg.values.append(kvp)

    kvp = kvpa()
    kvp.key = "tablet_url"
    kvp.value = image_url
    msg.values.append(kvp)
    rospy.logdebug("Sending CA_info (%s)" % msg_name)
    return msg