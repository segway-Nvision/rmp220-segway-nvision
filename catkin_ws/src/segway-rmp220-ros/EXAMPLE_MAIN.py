"""--------------------------------------------------------------------
COPYRIGHT 2013 SEGWAY Inc.

Software License Agreement:

The software supplied herewith by Segway Inc. (the "Company") for its 
RMP Robotic Platforms is intended and supplied to you, the Company's 
customer, for use solely and exclusively with Segway products. The 
software is owned by the Company and/or its supplier, and is protected 
under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Segway products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   EXAMPLE_MAIN.py

 \brief  This is an example on how to create a thread to talk to the RMP
         and handle various feedback

 \Platform: Cross Platform
--------------------------------------------------------------------"""
from system_defines import *
from rmp_interface import RMP
from user_event_handlers import RMPEventHandlers
import sys,time,threading,Queue

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


"""
Define the main function for the example. It creates a thread to run RMP and handles
passing the events to the user defined handlers in user_event_handlers.py
"""
def rmp_thread():
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
        while not out_flags.empty():
            EventHandler.handle_event[out_flags.get()]()
   	pass 
    """
    Generate a goto standby event
    """
    #EventHandler.GotoStandby()
    EventHandler.GotoBalance()
    
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
            
"""
This runs everything
"""
rmp_thread()    
    
