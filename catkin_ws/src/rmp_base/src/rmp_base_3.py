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
import multiprocessing


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
    def __init__(self, linear_vel, angular_vel):
        # Save the name of the node
        self.node_name = rospy.get_name()

        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        # Setup publishers
        self.pub_pose = rospy.Publisher("~pose",String, queue_size=1)

        self.lock = threading.Lock()

        # Setup subscriber
        self.subVelCmd = rospy.Subscriber("/rmp220/base/vel_cmd", TwistStamped, self.cbTopic, callback_args = (self.lock, linear_vel, angular_vel))
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)#publish robot status

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        #thread lock for control traffic flow for subscriber
        #self.thread_lock = threading.Lock()
        #self.active = True
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    def cbTopic(self, msg, args):
        #rospy.loginfo("[%s] %s" %(self.node_name,msg.data))
        #skip "if not" session if "self.active == true"
        #if not self.active:
                    #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z))
                    #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z))
                    #rospy.loginfo("From Houston!")
                    #alive=threading.active_count()
                    #rospy.loginfo("alive thread num: %d"%alive)
        #            return

        #keep return until, ignore thread approach

        lock        = args[0]
        linear_vel  = args[1]
        angular_vel = args[2]

        thread = threading.Thread(target=self.set_velocity,args=(msg, lock, linear_vel, angular_vel))
        thread.setDaemon(True)
        thread.start()

    def set_velocity(self, msg, lock, linear_vel, angular_vel):
        #self.active = False
        #check for thread time out
        #time_started = time.time()
        #self.thread_lock.acquire()

        #check for thread time out
        #thread_time_out = 2
        #if time.time() > time_started + thread_time_out:
        #    self.thread_lock.release()
        #    return

        lock.acquire()
        rospy.loginfo("velocity Components: [%f, %f]"%(msg.twist.linear.x, msg.twist.angular.z))
        if (msg.twist.linear.x)!=0.0:
            linear_vel = msg.twist.linear.x
        elif (msg.twist.angular.z)!=0.0:
            angular_vel = msg.twist.angular.z
        lock.release()

    def cbTimer(self,event):
        #singer = HelloGoodbye()
        # Simulate hearing something
        EventHandler.handle_event[RMP_RSP_DATA_RDY]()
        msg = String()
        msg.data = singer.sing("rmp_base")
        self.pub_pose.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

def send_velocity(linear_vel, angular_vel):
    #linear_vel  = args[0]
    #angular_vel = args[1]
    print("Send v = [%f, %f]"%(linear_vel, angular_vel))
    EventHandler.Send_MotionCmd(linear_vel, angular_vel)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('rmp_base', anonymous=False)
    #alive=threading.active_count()
    #rospy.loginfo("alive thread num: %d"%alive) //when init the node alive thread is 4
    #setup communication thread
    rsp_queue = multiprocessing.Queue()
    cmd_queue = multiprocessing.Queue()
    in_flags  = multiprocessing.Queue()
    out_flags = multiprocessing.Queue()
    linear_vel = 0.0
    angular_vel = 0.0
    my_thread = threading.Thread(target=RMP, args=(rmp_addr,rsp_queue,cmd_queue,in_flags,out_flags,UPDATE_DELAY_SEC,LOG_DATA))
    my_thread.setDaemon(True)
    my_thread.start()

    #svreate event handler and set to BALANCE MODE
    EventHandler = RMPEventHandlers(cmd_queue,rsp_queue,in_flags)

    #EventHandler.GotoTractor()
    #go to BALANCE
    EventHandler.GotoBalance()
    rospy.loginfo("finish initialize!")

    # Create the NodeName object
    node = rmp_base(linear_vel, angular_vel)

    while my_thread.isAlive():
        timer = threading.Timer(0.33, send_velocity, args = [linear_vel, angular_vel])
        timer.start()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
    sys.exit()


    #Noriginal thread number after node init 4, after setup communication thread 5, after setup rmp_node sub,pub 7
    #find out the issue for delay
