#!/usr/bin/env python
"""--------------------------------------------------------------------
Copyright (c) 2019, Tencent Technology inc.

    ____        __          __  _              _  __    __          __  
   / __ \____  / /_  ____  / /_(_)_________   | |/ /   / /   ____ _/ /_ 
  / /_/ / __ \/ __ \/ __ \/ __/ / ___/ ___/   |   /   / /   / __ `/ __ \
 / _, _/ /_/ / /_/ / /_/ / /_/ / /__(__  )   /   |   / /___/ /_/ / /_/ /
/_/ |_|\____/_.___/\____/\__/_/\___/____/   /_/|_|  /_____/\__,_/_.___/ 


All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.
      
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\Author Longfei Zhao
\brief  Demo to control a cart via serial port
\Platform: Linux/ROS Kinetic
--------------------------------------------------------------------"""

import rospy
from std_msgs.msg import String, UInt8MultiArray
from bitstring import BitArray


def pack_wcmd_bytes(f1, f2):
    pub_msg_buf = [0x00] * 19
    pub_msg_buf[0] = 0x05 # START
    pub_msg_buf[1] = 0x01 # ID
    pub_msg_buf[2] = 0x10 # FC
    pub_msg_buf[3] = 0x00 # ADDR
    pub_msg_buf[4] = 0x0C # ADDR
    pub_msg_buf[5] = 0x00 # NUMB
    pub_msg_buf[6] = 0x08 # NUMB

    pub_msg_buf[17] = 0x0D # NUMB
    pub_msg_buf[18] = 0x0A # NUMB

    # fill DATA
    f1_str = str(BitArray(float=f1, length=32))
    exec('pub_msg_buf[7]  = 0x'+f1_str[2:4])
    exec('pub_msg_buf[8]  = 0x'+f1_str[4:6])
    exec('pub_msg_buf[9]  = 0x'+f1_str[6:8])
    exec('pub_msg_buf[10] = 0x'+f1_str[8:10])

    f2_str = str(BitArray(float=f2, length=32))
    exec('pub_msg_buf[11] = 0x'+f2_str[2:4])
    exec('pub_msg_buf[12] = 0x'+f2_str[4:6])
    exec('pub_msg_buf[13] = 0x'+f2_str[6:8])
    exec('pub_msg_buf[14] = 0x'+f2_str[8:10])

    # fill CRC, for now just fill dummy 00
    exec('pub_msg_buf[15] = 0x00')
    exec('pub_msg_buf[16] = 0x00')

    return pub_msg_buf


def pack_rcmd_bytes():
    pub_msg_buf = [0x00] * 19
    pub_msg_buf[0] = 0x05 # START
    pub_msg_buf[1] = 0x01 # ID
    pub_msg_buf[2] = 0x03 # FC
    pub_msg_buf[3] = 0x00 # ADDR
    pub_msg_buf[4] = 0x01 # ADDR
    pub_msg_buf[5] = 0x00 # NUMB
    pub_msg_buf[6] = 0x34 # NUMB

    pub_msg_buf[9] = 0x0D # STOP
    pub_msg_buf[10] = 0x0A # STOP

    # fill CRC, for now just fill dummy 00
    exec('pub_msg_buf[7] = 0x00')
    exec('pub_msg_buf[8] = 0x00')

    return pub_msg_buf


class Cart:
    def __init__(self):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_rz = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_rz = 0.0

        self.cmdvel_x = 0.0
        self.cmdvel_y = 0.0

        self.wheel_radius = 0.0
        
        self.pub_cmd_to_slave = rospy.Publisher('/serial_tx', UInt8MultiArray)

        self.sub_feedback_from_slave = rospy.Subscriber('/serial_rx', UInt8MultiArray, self.cb_feedback_from_slave)
        
        rospy.spin()


    # host asks to write to register
    def send_wheel_cmdvel(self, cmdvel_x, cmdvel_y):
        self.cmdvel_x = cmdvel_x
        self.cmdvel_y = cmdvel_y
        data_out = pack_wcmd_bytes(self.cmdvel_x, self.cmdvel_y)

        cmdvel_msg = UInt8MultiArray()
        cmdvel_msg.layout.dim = len(data_out)
        cmdvel_msg.data = data_out

        self.pub_cmd_to_slave.publish(cmdvel_msg)


    # host asks to read register
    def request_robot_status(self):
        data_out = pack_rcmd_bytes()

        rsqvel_msg = UInt8MultiArray()
        rsqvel_msg.layout.dim = len(data_out)
        rsqvel_msg.data = data_out

        self.pub_cmd_to_slave.publish(rsqvel_msg)

    # Parse feedback from slave, and check whether data is valided.
    def cb_feedback_from_slave(self, msg):
        pass



if __name__ == "__main__":
    print('start cart control!')
    rospy.init_node('cart_control', log_level=rospy.INFO)

    cart = Cart()
    cart.send_wheel_cmdvel(0.1, 0.1)

    print('velocity of cart is velx: ', cart.vel_x, ', vely: ', cart.vel_y)
    print('estimated pose of cart is posex: ', cart.pose_x, ', posey: ', cart.pose_y, ', rotation along z is: ', cart.pose_rz)




