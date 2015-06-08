#!/usr/bin/env python
# encoding: utf-8
"""Module to turn the servo off"""

from mxt_protocol import RobotConnection, MXT_FFI


def get_servo_off_cmd():
    """Prepare the END command, which in turn turns off the servo
    :returns: command data

    """
    _mxt_cmd = MXT_FFI.new("MXTCMD *")
    _mxt_cmd.Command = 255
    return MXT_FFI.buffer(_mxt_cmd)


def main():
    """Establish connection and send the END command"""
    #mxt = RobotConnection("localhost")
    mxt = RobotConnection()
    mxt.mxt_connect()
    cmd = get_servo_off_cmd()
    mxt.mxt_send(cmd)

if __name__ == '__main__':
    main()
    pass
