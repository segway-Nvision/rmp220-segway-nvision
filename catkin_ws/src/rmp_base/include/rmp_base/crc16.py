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
demands,liabilities or expenses, including reasonable attorneys fees, incurred 
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
 
 \file   crc16.py

 \brief  This module contains a collection of functions for calculating
         a CRC-16.

 \Platform: Cross Platform
--------------------------------------------------------------------"""

"""
CRC16 defines
"""
CRC_ADJUSTMENT = 0xA001;
CRC_TABLE_SIZE = 256;
INITIAL_CRC = 0;

HIGH_BYTE_MASK  = 0xFF00;
LOW_BYTE_MASK   = 0x00FF;
MOVE_BYTE_SHIFT = 8;
LS_BIT          = 0x0001;
BITS_PER_BYTE   = 8;

crc_table= [0]*CRC_TABLE_SIZE;

"""
This creates the CRC table
"""
def generate_crc_table():

    for x in range(0,CRC_TABLE_SIZE):

        table_value = 0;
        k = x;

        for j in range(0,BITS_PER_BYTE):
            if (((table_value ^ k) & LS_BIT) == LS_BIT):
                table_value = (table_value >> 1) ^ CRC_ADJUSTMENT;
            else:
                table_value >>= 1;
            k >>= 1;

        crc_table[x] = table_value;

"""
This computes an updated CRC 16 given the current value of the CRC 16 and
a new data byte.
"""
def calculate_crc_16(old_crc,new_word):

    temp = old_crc ^ new_word;
    new_crc = (old_crc >> MOVE_BYTE_SHIFT) ^ crc_table[temp & LOW_BYTE_MASK];

    return new_crc;

"""
This computes the CRC for a buffer passed in
"""
def compute_buffer_crc(byte_buffer, bytes_in_buffer):
    crc_index = bytes_in_buffer - 2;
    new_crc = 0;

    """
    We'll loop through each word of the message and update
    the CRC.  Start with the value chosen for CRC initialization.
    """
    for x in range(0,crc_index):
        """
        Now we'll send each byte to the CRC calculation.
        """
        new_crc = calculate_crc_16(new_crc, byte_buffer[x]);


    byte_buffer[crc_index]   = int((new_crc & HIGH_BYTE_MASK) >> MOVE_BYTE_SHIFT);
    byte_buffer[crc_index+1] = int((new_crc & LOW_BYTE_MASK));
    
"""
This computes the CRC for a buffer passed in and checks it against
the one contained in the buffer
"""
def buffer_crc_is_valid(byte_buffer, bytes_in_buffer):
    
    crc_index = bytes_in_buffer - 2;
    new_crc = 0;
    success = False;

    """
    We'll loop through each word of the message and update
    the CRC.  Start with the value chosen for CRC initialization.
    """
    for x in range(0,crc_index):
        new_crc = calculate_crc_16(new_crc, byte_buffer[x]);

    """
    The new CRC is checked against that stored in the buffer.
    """
    received_crc = ((byte_buffer[crc_index] << MOVE_BYTE_SHIFT) & HIGH_BYTE_MASK);
    received_crc |= (byte_buffer[crc_index+1] & LOW_BYTE_MASK);
    
    if (received_crc == new_crc):
        success = True;
    else:
        success = False;

    return (success);

