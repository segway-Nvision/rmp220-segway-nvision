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
 
 \file   rmp_config_params.py

 \brief  Defines the user configurable parameters for loading at
         runtime

 \Platform: Cross-platform
--------------------------------------------------------------------"""
from system_defines import *
from utils import *

"""
The default values uncomment this section and remove the one below
to use the defaults

my_velocity_limit_mps             = DEFAULT_MAXIMUM_VELOCITY_MPS
my_accel_limit_mps2               = DEFAULT_MAXIMUM_ACCELERATION_MPS2
my_decel_limit_mps2               = DEFAULT_MAXIMUM_DECELERATION_MPS2
my_dtz_rate_mps2                  = DEFAULT_MAXIMUM_DTZ_DECEL_RATE_MPS2
my_coastdown_accel_mps2           = DEFAULT_COASTDOWN_ACCEL_MPS2
my_yaw_rate_limit_rps             = DEFAULT_MAXIMUM_YAW_RATE_RPS
my_yaw_accel_limit_rps2           = DEFAULT_MAX_YAW_ACCEL_RPS2
my_tire_diameter_m                = I2_TIRE_DIAMETER_M
my_wheel_base_length_m            = DEFAULT_WHEEL_BASE_LENGTH_M
my_wheel_track_width_m            = I2_WHEEL_TRACK_WIDTH_M
my_gear_ratio                     = I2_TRANSMISSION_RATIO
my_config_bitmap                  = DEFAULT_CONFIG_BITMAP
my_ip_address                     = DEFAULT_IP_ADDRESS
my_port_num                       = DEFAULT_PORT_NUMBER
my_subnet_mask                    = DEFAULT_SUBNET_MASK
my_gateway                        = DEFAULT_GATEWAY
my_user_defined_feedback_bitmap_1 = DEFAULT_USER_FB_1_BITMAP
my_user_defined_feedback_bitmap_2 = DEFAULT_USER_FB_1_BITMAP
my_user_defined_feedback_bitmap_3 = DEFAULT_USER_FB_1_BITMAP
my_user_defined_feedback_bitmap_4 = DEFAULT_USER_FB_1_BITMAP
"""
my_velocity_limit_mps             = 1.0
my_accel_limit_mps2               = 0.981
my_decel_limit_mps2               = 0.981
my_dtz_rate_mps2                  = 0.981
my_coastdown_accel_mps2           = 0.1962
my_yaw_rate_limit_rps             = 1.0
my_yaw_accel_limit_rps2           = 1.0
my_tire_diameter_m                = I2_TIRE_DIAMETER_M
my_wheel_base_length_m            = DEFAULT_WHEEL_BASE_LENGTH_M
my_wheel_track_width_m            = I2_WHEEL_TRACK_WIDTH_M
my_gear_ratio                     = I2_TRANSMISSION_RATIO
my_config_bitmap                  = DEFAULT_CONFIG_BITMAP
my_ip_address                     = DEFAULT_IP_ADDRESS
my_port_num                       = DEFAULT_PORT_NUMBER
my_subnet_mask                    = DEFAULT_SUBNET_MASK
my_gateway                        = DEFAULT_GATEWAY
my_user_defined_feedback_bitmap_1 = DEFAULT_USER_FB_1_BITMAP
my_user_defined_feedback_bitmap_2 = DEFAULT_USER_FB_2_BITMAP
my_user_defined_feedback_bitmap_3 = DEFAULT_USER_FB_3_BITMAP
my_user_defined_feedback_bitmap_4 = DEFAULT_USER_FB_4_BITMAP

"""
Modify above not below, this section is populated with those values and should not be changed
this variable is used in other files; size, order and contents matter
"""
CONFIG_PARAMS = [
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_VELOCITY,convert_float_to_u32(my_velocity_limit_mps)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_ACCELERATION,convert_float_to_u32(my_accel_limit_mps2)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_DECELERATION,convert_float_to_u32(my_decel_limit_mps2)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_DTZ_DECEL_RATE,convert_float_to_u32(my_dtz_rate_mps2)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_COASTDOWN_ACCEL,convert_float_to_u32(my_coastdown_accel_mps2)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_TURN_RATE,convert_float_to_u32(my_yaw_rate_limit_rps)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_MAXIMUM_TURN_ACCEL,convert_float_to_u32(my_yaw_accel_limit_rps2)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_TIRE_DIAMETER,convert_float_to_u32(my_tire_diameter_m)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_WHEEL_BASE_LENGTH,convert_float_to_u32(my_wheel_base_length_m)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_WHEEL_TRACK_WIDTH,convert_float_to_u32(my_wheel_track_width_m)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_TRANSMISSION_RATIO,convert_float_to_u32(my_gear_ratio)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_INPUT_CONFIG_BITMAP,my_config_bitmap],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_IP_ADDRESS,dottedQuadToNum(my_ip_address)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_PORT_NUMBER,my_port_num],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_SUBNET_MASK,dottedQuadToNum(my_subnet_mask)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_ETH_GATEWAY,dottedQuadToNum(my_gateway)],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_1_BITMAP,my_user_defined_feedback_bitmap_1],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_2_BITMAP,my_user_defined_feedback_bitmap_2],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_3_BITMAP,my_user_defined_feedback_bitmap_3],
     [RMP_CFG_CMD_ID,RMP_CMD_SET_USER_FB_4_BITMAP,my_user_defined_feedback_bitmap_4]]
