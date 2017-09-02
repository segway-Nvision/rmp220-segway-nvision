#!/usr/bin/env python
import rospy
from system_defines import *
from user_event_handlers import RMPEventHandlers
import sys,time,threading,Queue
from rmp_interface import RMP #Imports module. Not limited to modules in this pkg.
from std_msgs.msg import String #Imports msg

#for rmp220
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu,JointState
from rmp_msgs.msg import BoolStamped,AudioCommand,FaultStatus



"""
Define the update delay or update period in seconds. Must be greater
than the minimum of 0.01s
"""
UPDATE_DELAY_SEC = 0.05

"""
Define whether to log the output data in a file. This will create a unique CSV
file in ./RMP_DATA_LOGS with the filename containing a time/date stamp
"""
LOG_DATA = True

"""
The platform address may be different than the one in your config
(rmp_config_params.py). This would be the case if you wanted to update
ethernet configuration. If the ethernet configuration is updated the
system needs to be power cycled for it to take effect and this should
be changed to match the new values you defined in your config
"""
rmp_addr = ("192.168.0.40",8080) #this is the default value and matches the config


class rmp_base(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        rospy.loginfo("setting balance mode!")

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~pose",String, queue_size=1)
        # Setup subscriber
        self.sub_topic_b = rospy.Subscriber("/rmp220/base/vel_cmd", TwistStamped, self.cbTopic)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        #self.thread_lock = threading.Lock()
        self.active = False
        #self.rmp_thread()//start everything
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):
        #rospy.loginfo("[%s] %s" %(self.node_name,msg.data))
        #skip "if not" part if self.active == true
        if not self.active:
                    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z))
                    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z))
                    rospy.loginfo("From Houston!")
        return

	#	thread = threading.Thread(target=self.rmp_thread,args=(msg,))
	#	thread.setDaemon(True)
    #    thread.start()


    def cbTimer(self,event):
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("rmp_base")
        self.pub_topic_a.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

    def rmp_thread(self):

        """
        Create and response and command queue. The responses will be in the form of
        a dictionary containing the vaiable name as the key and a converted value
        the names are defined in the feedback_X_bitmap_menu_items dictionaries if a particular
        variable is of interest
        """
        rsp_queue = Queue.Queue()
        cmd_queue = Queue.Queue()
        in_flags  = Queue.Queue()
        out_flags = Queue.Queue()

        """
        Create the thread to run RMP
        """
        my_thread = threading.Thread(target=RMP, args=(rmp_addr,rsp_queue,cmd_queue,in_flags,out_flags,UPDATE_DELAY_SEC,LOG_DATA))
        my_thread.daemon = True
        my_thread.start()

        """
        Initialize my event handler class
        """
        EventHandler = RMPEventHandlers(cmd_queue,rsp_queue,in_flags)

        """
        -------------------------------------------------------------------------------
        User loop starts here modify to make it do what you want.

        You can pipe std_in from another application to the command queue and the response to std out or
        let the event handlers define everything. That is up to the user. In this example we transition modes,
        send motion commands (zeroed), play audio songs, and print the response dictionary. The application
        terminates the thread and exits when all the songs have been played. It is just an example of how to
        spawn a RMP thread, handle events, and send/receive data
        -------------------------------------------------------------------------------
        """

        """
        Generate a goto tractor event
        """
        #EventHandler.GotoTractor()
        #EventHandler.GotoStandby()
        EventHandler.GotoBalance()

        """
        Run until signaled to stop
        Perform the actions defined based on the flags passed out
        """
        while (True == EventHandler._continue):
            EventHandler.handle_event[RMP_ROTATE]()
            EventHandler.handle_event[RMP_RSP_DATA_RDY]()
            time.sleep(1.0)
            EventHandler.handle_event[RMP_ZERO]()
            EventHandler.handle_event[RMP_RSP_DATA_RDY]()
            time.sleep(1.0)
            EventHandler.handle_event[RMP_FORWARD]()
            EventHandler.handle_event[RMP_RSP_DATA_RDY]()
            time.sleep(1.0)
            pass
            #while not out_flags.empty():
            #  EventHandler.handle_event[out_flags.get()]()
        """
        Generate a goto standby event
        """

        """
        Allow enough time for the command to be sent
        then send the signal to kill the thread
        """
        time.sleep(1.0)
        in_flags.put(RMP_KILL)

        """
        Wait for the thread to die
        """
        while my_thread.isAlive():
            pass

        """
        Exit main
        """
        sys.exit()
if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('rmp_base', anonymous=False)

    # Create the NodeName object
    node = rmp_base()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
