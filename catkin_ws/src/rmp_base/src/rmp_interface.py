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
 
 \file   rmp_interface.py

 \brief  This module contains the interface to the RMP

 \Platform: Cross Platform
--------------------------------------------------------------------"""
from crc16  import *
from utils import *
from system_defines import *
from rmp_config_params import *
from user_event_handlers import *
from io_eth_cmd import IO_ETHERNET
import time,os,sys


"""
Define the variable for fixed point Q15 format for
converting certain parameters for OMNI
"""
Q15             = 32767

"""
Main class for the RMP interface
"""
class RMP:
    def __init__(self,rmp_addr,rsp_queue,cmd_queue,in_flags,out_flags,update_rate=MIN_UPDATE_PERIOD_SEC,log_data=False):        
        """
        generate the CRC table
        """
        generate_crc_table()        
        
        """
        Create a dictionary for the feedback
        """
        self.user_defined_feedback = dict()
        
        """
        Indicates a logfile has been created
        """
        self.logfile_started = False
        
        """
        Initialize the bitmaps and create a dictionary for holding the
        user defined feedback data
        """
        self.bitmap = [0]*4
        self.bitmap[0] = CONFIG_PARAMS[RMP_CMD_SET_USER_FB_1_BITMAP-1][2]
        self.bitmap[1] = CONFIG_PARAMS[RMP_CMD_SET_USER_FB_2_BITMAP-1][2]
        self.bitmap[2] = CONFIG_PARAMS[RMP_CMD_SET_USER_FB_3_BITMAP-1][2]
        self.bitmap[3] = CONFIG_PARAMS[RMP_CMD_SET_USER_FB_4_BITMAP-1][2]
        
        """
        Create arrays of bitmaps defining the type of feedback contained
        in each UDFB
        """
        self.fp_mask  = [RMP_FLOATING_POINT_FEEDBACK_1_MASK,
                         RMP_FLOATING_POINT_FEEDBACK_2_MASK,
                         RMP_FLOATING_POINT_FEEDBACK_3_MASK,
                         RMP_FLOATING_POINT_FEEDBACK_4_MASK]
        
        self.hex_mask  = [RMP_HEX_FEEDBACK_1_MASK,
                          RMP_HEX_FEEDBACK_2_MASK,
                          RMP_HEX_FEEDBACK_3_MASK,
                          RMP_HEX_FEEDBACK_4_MASK]
        
        self.ip_mask  = [RMP_IP_FEEDBACK_1_MASK,
                         RMP_IP_FEEDBACK_2_MASK,
                         RMP_IP_FEEDBACK_3_MASK,
                         RMP_IP_FEEDBACK_4_MASK]
        
        """
        Create a header dictionary array
        """
        self.hdr_dicts = [feedback_1_bitmap_menu_items,
                          feedback_2_bitmap_menu_items,
                          feedback_3_bitmap_menu_items,
                          feedback_4_bitmap_menu_items]
        
        """
        Update the local feedback dictionary
        """
        self.update_feedback_dict(None,True)
        
        """
        Get the RMP address
        """
        self.comm = IO_ETHERNET(rmp_addr)
        
        self.in_flags = in_flags
        self.out_flags = out_flags
        
        if (update_rate >= MIN_UPDATE_PERIOD_SEC):
            self.delay = update_rate
        else:
            print "Bad Update Period needs to be longer than 0.01s....."
            print "exiting......"
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
                    
        if (False == self.comm.success):
            print "Could not connect to RMP UDP socket....."
            print "exiting......"
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
            
        if (False == self.set_and_verify_config_params(CONFIG_PARAMS)):
            print "Could not configure RMP......"
            print "exiting......"
            self.out_flags.put(RMP_INIT_FAILED)
            self.Close()
        
        """
        Get the queues and the time stamps now that initialization was successful
        """
        self.cmd_queue = cmd_queue
        self.rsp_queue = rsp_queue
        self.last_update_time = time.time()
        self.log_file_start_time = time.time()
        
        """
        If the caller wants to log data check for the log directory, create the file and write
        the header.
        """
        self.log_data = log_data
        if self.log_data:
            dirpath = os.getcwd()+"/RMP_DATA_LOGS"
            if (False == os.path.exists(dirpath)):
                os.mkdir(dirpath)
            filename = dirpath + "/" + "RMP_DATA_LOG_" + time.strftime("%m%d%Y_%H%M%S") + ".csv"
            self.logfile = open(filename,'w')
            self.logfile.write(','.join(self.log_file_header))
            self.logfile_started = True 
            
        """
        Run the thread
        """
        self.run()
        
    def run(self):
        while True:
            
            """
            Check the flags coming in. Presently there is only a kill.
            """
            while not self.in_flags.empty():
                if (RMP_KILL == self.in_flags.get()):
                    print "RMP thread has been killed by application......"
                    print "exiting........."
                    self.out_flags.put(RMP_IS_DEAD)
                    self.Close()
                    sys.exit() 

            """
            If enough time has passed since the last transmit, check the command
            queue and send the next command. Then set the TX_RDY flag so the parent
            knows that the queue can be repopulated
            """
            if ((time.time() - self.last_update_time) > self.delay):
                if not self.cmd_queue.empty():
                    self.update_rmp_commands(self.cmd_queue.get())
                    
                self.out_flags.put(RMP_TX_RDY)
                self.last_update_time = time.time()

            """
            Check for data each time and put it in the queue is it exists.
            """
            data = self.comm.Receive(self.expected_items)
            if (self.update_feedback_dict(data,False)):
                self.rsp_queue.put(self.user_defined_feedback)
                self.out_flags.put(RMP_RSP_DATA_RDY)

    def set_and_verify_config_params(self,config):
        """
        The commands that force the feedback array to just contain the configurable elements
        but not update the UDFB to do so. This allows the user to verify the configuration
        while it is changing.
        """
        force_nvm_feedback = [RMP_CFG_CMD_ID,RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS,1]
        set_user_feedback = [RMP_CFG_CMD_ID,RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS,0]
        
        
        """
        Start by sending the force config feedback command and check for the
        appropriate response
        """
        attempts = 0
        success = False
        while ((False == success) and (attempts<10)):
            self.update_rmp_commands(force_nvm_feedback)
            time.sleep(0.05)
            loaded_params = self.comm.Receive(FORCED_CONFIG_FEEDBACK_ITEMS)
            if (None != loaded_params):
                success = True
            else:
                attempts += 1
        
        """
        If the command to force the config feedback was successful check to see if there
        are non-matching configuration parameters
        """
        if (False == success):
            print "Could not set RMP_CMD_FORCE_CONFIG_FEEDBACK_BITMAPS....."
            print "The platform did not respond, ensure it is operational and the IP address is correct...."
            return False
        else:
            non_matching_params = []
            for i in range(0,NUMBER_OF_NVM_CONFIG_PARAMS):
                if (loaded_params[i] != config[i][2]):
                    non_matching_params.append(i)
        
        """
        Load each configuration parameter which does not match the configuration file
        """
        attempts = 0    
        for i in range(0,len(non_matching_params)):
            idx = non_matching_params[i]
            success = False
            while ((False == success) and (attempts<10)):
                attempts+=1
                self.update_rmp_commands(config[idx])
                time.sleep(0.05)
                loaded_params = self.comm.Receive(FORCED_CONFIG_FEEDBACK_ITEMS)
                if (None != loaded_params):
                    if (loaded_params[idx] == config[idx][2]):
                        success = True
                        attempts = 0
            if (False == success):
                break
        
        """
        Notify the user if we could not set the parameter
        """
        if (False == success):
            print "Could not set param %(1)s....." %{"1":config_param_dict[idx+1]}
            print "The parameter is likely not valid, check it in rmp_config_params.py"
            return False
        
        """
        Switch back to the user feedback array and make sure we get an appropriate response
        """
        attempts = 0
        success = False
        while ((False == success) and (attempts<10)):
            self.update_rmp_commands(set_user_feedback)
            time.sleep(0.05)
            data = self.comm.Receive(self.expected_items)
            if (None != data):
                success = True
            else:
                attempts += 1
                    
        """
        Could not reset the feedback 
        """
        if (False == success):
            print "Could not set user defined feedback" %{"1":idx+1}
            print "The platform did not respond, "
            return False
        
        return True
    
    def update_rmp_commands(self,input_cmd):
       
        """
        Populate the message to the RMP platform if it is not a
        valid format return False
        """
        try:
            cmds = [0]*3
            cmds[0] = input_cmd[0]
            send_cmd = True
            
            if (cmds[0] == RMP_OMNI_MOTION_CMD_ID):
                vel_cmd = int((input_cmd[1] * Q15))
                yaw_cmd = int((input_cmd[2] * Q15))
                cmds[1]= (((vel_cmd << 16) & 0xFFFF0000) | (yaw_cmd & 0x0000FFFF))
                cmds[2] = int(convert_float_to_u32(input_cmd[3]))
            elif (cmds[0] == RMP_MOTION_CMD_ID):
                cmds[1] = int(convert_float_to_u32(input_cmd[1]))
                cmds[2] = int(convert_float_to_u32(input_cmd[2]))
            elif (cmds[0] == RMP_CFG_CMD_ID):
                cmds[1] = int(input_cmd[1])
                cmds[2] = int(input_cmd[2])
            else:
                send_cmd = False
        except:
            send_cmd = False
            
        """
        If we have a valid command send it 
        """
        if (True == send_cmd):
                
            output = self.Convert_RMP_Cmds_for_Serial_Interface(cmds)
            self.comm.Send(output)
            
        return send_cmd
    
    def update_feedback_dict(self,data=None,init=False):
        ret = True
        
        """
        Determine the way the variables should be converted to human readable form. There are
        four ways, floating point, hex, IP strings, and the default is integer
        """
        item = 0
        if init:
            self.log_file_header = ["Time_stamp"]
            for x in range(0,4):                
                for i in range(0,32):
                    if (self.bitmap[x] & (1<<i)):
                        if (self.fp_mask[x] & (1<<i)):
                            self.log_file_header.append(self.hdr_dicts[x][(1<<i)])
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = 0.0
                        elif (self.hex_mask[x] & (1 << i)):
                            self.log_file_header.append(self.hdr_dicts[x][(1<<i)])
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = hex(0)
                        elif (self.ip_mask[x] & (1 << i)):
                            self.log_file_header.append(self.hdr_dicts[x][(1<<i)])
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = numToDottedQuad(0)
                        else:
                            self.log_file_header.append(self.hdr_dicts[x][(1<<i)])
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = 0
                        
                        item +=1
            """
            Add one for the CRC
            """
            self.expected_items = item+1
            
            """
            Append a line end to the logfile header
            """
            self.log_file_header.append("\n")
            
        elif (data != None):
            """
            If we have data convert it and add it to the dictionary
            """
            temp = []
            temp.append(str(time.time() - self.log_file_start_time))
            self.user_defined_feedback.clear();
            for x in range(0,4):
                for i in range(0,32):
                    if (self.bitmap[x] & (1<<i)): 
                        if (self.fp_mask[x] & (1<<i)):
                            temp.append(str(round(convert_u32_to_float(data[item]),3)))
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = round(convert_u32_to_float(data[item]),3);
                        elif (self.hex_mask[x] & (1 << i)):
                            temp.append("0x%(1)08X" %{"1":data[item]})
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = hex(data[item]);
                        elif (self.ip_mask[x] & (1 << i)):
                            temp.append(str(numToDottedQuad(data[item])))
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = numToDottedQuad(data[item]);
                        else:
                            temp.append(str(int(data[item])))
                            self.user_defined_feedback[self.hdr_dicts[x][(1<<i)]] = int(data[item]);
                        
                        item += 1
            
            """
            Log the data if called for
            """
            if(True == self.logfile_started):
                temp.append("\n")
                self.logfile.write(",".join(temp))
        else:
            ret = False
        
        return ret
            
    def Close(self):
        if (True == self.logfile_started):
            self.logfile.close()
        self.comm.Close()
        sys.exit()
            
    def Convert_RMP_Cmds_for_Serial_Interface(self,cmds):
        """
        Convert a set of commands for the UDP Ethernet interface
        """
        rmp_cmd = [0]*NUM_USB_ETH_BYTES;
        
        rmp_cmd[RMP_USB_ETH_CAN_ID_HIGH_INDEX] = int((cmds[0] & 0xFF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_ID_LOW_INDEX]  = int((cmds[0] & 0x00FF))
        rmp_cmd[RMP_USB_ETH_CAN_DATA_0_INDEX]  = int((cmds[1] & 0xFF000000) >> 24)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_1_INDEX]  = int((cmds[1] & 0x00FF0000) >> 16)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_2_INDEX]  = int((cmds[1] & 0x0000FF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_3_INDEX]  = int((cmds[1] & 0x000000FF))
        rmp_cmd[RMP_USB_ETH_CAN_DATA_4_INDEX]  = int((cmds[2] & 0xFF000000) >> 24)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_5_INDEX]  = int((cmds[2] & 0x00FF0000) >> 16)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_6_INDEX]  = int((cmds[2] & 0x0000FF00) >> 8)
        rmp_cmd[RMP_USB_ETH_CAN_DATA_7_INDEX]  = int((cmds[2] & 0x000000FF))
        
        """
        Compute the CRC for the command 
        """
        compute_buffer_crc(rmp_cmd,NUM_USB_ETH_BYTES)
        
        """
        Convert the string to char data and return it
        """
        rmp_cmd_chars = []
        for x in range(0,len(rmp_cmd)):
            rmp_cmd_chars.append(chr(rmp_cmd[x]))   
        
        output = ''.join(rmp_cmd_chars)
        
        return output        
        
        
    
