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

START_BYPE = 0x05
SLAVE_ID = 0x01
STOP_BYPE1 = 0x0D
STOP_BYPE2 = 0x0A


def pack_wcmd_bytes(f1, f2):
    pub_msg_buf = [0x00] * 19
    pub_msg_buf[0] = START_BYPE # START
    pub_msg_buf[1] = START_BYPE # ID
    pub_msg_buf[2] = 0x10 # FC
    pub_msg_buf[3] = 0x00 # ADDR
    pub_msg_buf[4] = 0x0C # ADDR
    pub_msg_buf[5] = 0x00 # NUMB
    pub_msg_buf[6] = 0x08 # NUMB

    pub_msg_buf[17] = STOP_BYPE1 # NUMB
    pub_msg_buf[18] = STOP_BYPE2 # NUMB

    # fill DATA
    # f1_str = str(bitstring.pack('>f',f1))
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
    pub_msg_buf[0] = START_BYPE # START
    pub_msg_buf[1] = START_BYPE # ID
    pub_msg_buf[2] = 0x03 # FC
    pub_msg_buf[3] = 0x00 # ADDR
    pub_msg_buf[4] = 0x01 # ADDR
    pub_msg_buf[5] = 0x00 # NUMB
    pub_msg_buf[6] = 0x34 # NUMB

    pub_msg_buf[9] = STOP_BYPE1 # STOP
    pub_msg_buf[10] = STOP_BYPE2 # STOP

    # fill CRC, for now just fill dummy 00
    exec('pub_msg_buf[7] = 0x00')
    exec('pub_msg_buf[8] = 0x00')

    return pub_msg_buf


def convert_hexstr_to_hexlist(hex_str):
    # strip off '0x' if existed
    if hex_str[0:2] == '0x':
        hex_str = hex_str[2:]
    
    bytes_list_size = 0
    if len(hex_str)%2 != 0:
        raise Exception('wrong hex string provided, the string must be even size')
    else:
        bytes_list_size = len(hex_str)/2
    
    bytes = [0x00]*bytes_list_size
    for i in range(bytes_list_size):
        bytes[i] = int(hex_str[2*i:2*i+2], 16)
        # print('i is ', i, ', and hex_str[2*i:2*i+2] is', hex_str[2*i:2*i+2])
    
    return bytes


def bytes_to_float(hex_str):
    if(len(hex_str) == 8):
        f = BitArray(hex=hex_str)
        return f.float
    else:
        rospy.logwarn('The float in the prototype must have the length of 4 bytes!')
        return 0.0


def float_equal(f1, f2):
    if abs(f1-f2)<=1e-6:
        return True
    else:
        return False


def check_crc16(bytes):
    return True
    pass


class Cart:
    def __init__(self):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_rz = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_rz = 0.0

        self.cmdvel_l = 0.0
        self.cmdvel_r = 0.0

        self.wheel_radius = 0.0
        
        self.pub_cmd_to_slave = rospy.Publisher('/serial_tx', UInt8MultiArray, queue_size=10)

        self.sub_feedback_from_slave = rospy.Subscriber('/serial_rx', UInt8MultiArray, self.cb_feedback_from_slave)
        
        # test cb_feedback_from_slave/{condition: bytes[2] == 0x10}
        self.send_wheel_cmdvel(3.5, -3.5)
        rospy.spin()


    # host asks to write to register
    def send_wheel_cmdvel(self, cmdvel_l, cmdvel_r):
        self.cmdvel_l = cmdvel_l
        self.cmdvel_r = cmdvel_r
        bytes_out = pack_wcmd_bytes(self.cmdvel_l, self.cmdvel_r)

        cmdvel_msg = UInt8MultiArray()
        cmdvel_msg.layout.dim = len(bytes_out)
        cmdvel_msg.data = bytes_out

        rospy.loginfo('send velocity command (cmdvel_l: %f, cmdvel_r: %f) to slave: ', self.cmdvel_l, self.cmdvel_r)
        self.pub_cmd_to_slave.publish(cmdvel_msg)


    # host asks to read register
    def request_robot_status(self):
        bytes_out = pack_rcmd_bytes()

        rsqvel_msg = UInt8MultiArray()
        rsqvel_msg.layout.dim = len(bytes_out)
        rsqvel_msg.bytes = bytes_out

        self.pub_cmd_to_slave.publish(rsqvel_msg)


    # Parse feedback from slave, and check whether data is valided.
    def cb_feedback_from_slave(self, msg):

        print('cb_feedback_from_slave have received package from the host.')
        # print('type(msg) is : ', type(msg), ', and the raw msg is: \n', msg)

        bytes_len = msg.layout.dim[0].size
        bytes_str = msg.data.encode('hex')
        bytes = convert_hexstr_to_hexlist(bytes_str)

        print('bytes_len is: ', bytes_len, ', bytes_str is: ', bytes_str, ' and bytes is: ', bytes)

        if bytes[0] != START_BYPE:
            # raise Exception("package received from host does not have the right START_BYTE")
            rospy.logwarn('package does not meet START_BYPE')
            return
        
        if bytes[1] == SLAVE_ID:
            if bytes[2] == 0x03:
                rospy.loginfo('received feedback from command: read register')
                if check_crc16(bytes):
                    vl_cmd = bytes_to_float(bytes[45:49])
                    vr_cmd = bytes_to_float(bytes[49:])
                else:
                    rospy.logwarn('check crc16 failed')

            elif bytes[2] == 0x83:
                rospy.logwarn('failed to receive feedback from command: read register')
                if check_crc16(bytes):
                    error_code = int(bytes_str[14:22], 16)
                    rospy.logwarn('error_code is %d', error_code)
                else:
                    rospy.logwarn('check crc16 failed')
                pass

            elif bytes[2] == 0x10:
                rospy.loginfo('received feedback from command: write to register')
                if check_crc16(bytes):
                    if float_equal(self.cmdvel_l, bytes_to_float(bytes_str[14:22])) and float_equal(self.cmdvel_r, bytes_to_float(bytes_str[22:30])):
                        rospy.loginfo('slave confirmed the velocity command')
                    else:
                        rospy.logwarn('slave does not match the sent velocity command')
                else:
                    rospy.logwarn('check crc16 failed')

            elif bytes[2] == 0x90:
                rospy.logwarn('failed to receive feedback from command: write to register')
                if check_crc16(bytes):
                    error_code = int(bytes_str[14:22], 16)
                    rospy.logwarn('error_code is %d', error_code)
                else:
                    rospy.logwarn('check crc16 failed')
                pass
            else:
                rospy.logwarn('unkown function code received')
            
        else:
            rospy.logwarn('package does not meet SLAVE_ID')


if __name__ == "__main__":
    print('start cart control!')
    rospy.init_node('cart_control', log_level=rospy.INFO)

    rospy.logwarn('-------------debug0')
    cart = Cart()


