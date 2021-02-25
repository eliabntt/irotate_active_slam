#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, REC Robotics Equipment Corporation GmbH, Planegg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import requests 
import sys
import rospy
from robotino_msgs.msg import AnalogReadings

# api-endpoint 
URL = "http://127.0.0.1/data/analoginputarray"
PARAMS = {'sid':'robotino_rest_node'} 

def talker():
	analog_readingsPub = rospy.Publisher('analog_readings', AnalogReadings, queue_size=1)
	rospy.init_node('robotino_analoginput', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		try:
			r = requests.get(url = URL, params = PARAMS)
			if r.status_code == requests.codes.ok:
				data = r.json() 
				rospy.loginfo(data)
				msg = AnalogReadings()
				msg.stamp = rospy.get_rostime()
				msg.values = data
				analog_readingsPub.publish(msg)
			else:
				rospy.logwarn("get from %s with params %s failed", URL, PARAMS)
		except requests.exceptions.RequestException as e:
			rospy.logerr("%s", e)
			pass
		rate.sleep()

if __name__ == '__main__':
	myargv = rospy.myargv(argv=sys.argv)
	if len(myargv)>1:
		URL = URL.replace("127.0.0.1",myargv[1])
	print("connecting to: ",URL)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
