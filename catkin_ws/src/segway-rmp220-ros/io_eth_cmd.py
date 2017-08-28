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
 
 \file   io_eth_cmd.py

 \brief  This module contains the ethernet UDP communication protocol

 \Platform: Cross platform
--------------------------------------------------------------------"""
from utils import convert_byte_data_to_U32
import socket

class IO_ETHERNET:
    def __init__(self,rmp_address):
        
        self.success = True
        
        """
        Initialize link parameters
        """
        my_address = ('',rmp_address[1])

        """
        Initialize the UDP connection
        """
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conn.setblocking(False)
        self.conn.bind(my_address)
        try:
            self.conn.connect(rmp_address)
        except:
            self.success = False

    def Send(self,data):
        try:
            self.conn.sendall(data)            
        except:
            pass
    
    def Receive(self,num_of_return):
        
        """
        The number of bytes expected is the number of 32-bit messages times
        the bytes per 32-bit word
        """    
        num_of_bytes = num_of_return * 4
        
        """
        Try receiving the data up to a maximum size. If it fails
        empty the data
        """
        try:
            data = self.conn.recv(1024)
        except:
            data = []
            
        """
        If the data is the length expected, convert the byte data to U32 data and return it.
        Otherwise return the None type.
        """
        if (len(data) == num_of_bytes):
            return_data = convert_byte_data_to_U32(data);
        else:
            return_data = None; 
            
        return return_data;

    def Close(self):
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        
