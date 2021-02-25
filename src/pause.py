import rospy
from std_srvs.srv import Empty

import time
a = rospy.client

time.sleep(10*60+3) #sleep 10 minutes
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
pause.call()
print("GAZEBO PAUSED, YOU CAN COPY YOUR FILES IF USEFUL")
