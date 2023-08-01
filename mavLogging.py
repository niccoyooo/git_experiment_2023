#!/usr/bin/env python

from pymavlink import mavutil
import serial 


class Bridge(object):
    """ MAVLink bridge

    Attributes:
        conn (TYPE): MAVLink connection
        data (dict): Deal with all data
    """
    import serial
    def __init__(self, device='/dev/serial0', baudrate=921600):
        """
        Args:
            device (str, optional): Input device
                https://ardupilot.github.io/MAVProxy/html/getting_started/starting.html#master
            baudrate (int, optional): Baudrate for serial communication
        """
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.data = {}
        

    def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data

    def get_all_msgs(self):
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        """ Update data dict
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            if msg.get_type() == 'HEARTBEAT':
                self.data[msg.get_type()] = msg.to_dict()
            elif msg.get_type() == 'ATTITUDE':
                self.data[msg.get_type()] = msg.to_dict()
            elif msg.get_type() == 'VFR_HUD':
                self.data[msg.get_type()] = msg.to_dict()
            elif msg.get_type() == 'SYSTEM_TIME':
                self.data[msg.get_type()] = msg.to_dict()

    def print_data(self):
        """ Debug function, print data dict
        """
        print(self.data)

    def set_rc_channel_pwm(self, id, pwm=1100):
        """ Set RC channel pwm value

        Args:
            id (TYPE): Channel id
            pwm (int, optional): Channel pwm value 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id] = pwm
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.


if __name__ == '__main__':
    #bridge = Bridge()
    bridge = Bridge(device='/dev/serial0')
    count = 2000
    while True:
        
        bridge.update()
        
        if count < 1:
            count = 2000
            bridge.print_data()
        else:
            count-=1
