#!/usr/bin/env python

import rospy
from neostate_msgs.msg import StatusLEDArray, StatusLED


class LEDArray(object):
    def __init__(self, num_leds):
        """Python ROS Interface for changing the LED colors and publishing it
        to the Arduino board.

        Args:
            num_leds: number of leds on board
        """
        if num_leds < 1:
            raise TypeError("Number of LEDs must be bigger than 0!")

        self.__num_of_leds = num_leds

        self.__led_status = [0] * self.__num_of_leds

        # Store RGB value for every LED
        self.__led_colors = [[0, 0, 0]] * self.__num_of_leds

        self.__msg_pub = rospy.Publisher(
            "/set_status_led_array", StatusLEDArray, queue_size=1, latch=True
        )

    @property
    def led_status(self):
        """Status of every LED.
        
        The index of the array corresponds to the led id.
      
        Status values:
            0: off
            -1: on
            >0: blinking at predefined (on board) frequency
        """
        return self.__led_status

    @property
    def led_colors(self):
        """Color of every LED.
        The index of the array corresponds to the led id.

        Color is stored as a list of RGB values which range from 
        0 (off) - 255 (very very bright). 
        """
        return self.__led_colors

    def get_num_leds(self):
        """Get number of leds on neopixel strip."""
        return self.__num_of_leds

    def set_led_status(self, ids, status, rgb_color):
        """Change the color of one or more LEDs.

        All arguments need to be passed as a list.

        Args:
            ids: ids of leds whose color should be changed
            status: status of led (on, blinking, off) to be set
            rgb_color: desired rgb value of led (zero if status is set to 0) 
        """
        #TODO(clanegge): Check if anything has changed and only publish if yes.
        #TODO(clanegge): Raise Error here
        try:
            if len(ids) is not len(status) is not len(rgb_color):
                rospy.logwarn(
                    "[Piksi Status Indicator]: "
                    + "Cannot assign values to ids as their array size is not the same."
                    + "Not changing LED status."
                )
                return
            elif len(ids) > len(self.__led_status):
                rospy.logwarn(
                    "[Piksi Status Indicator]:"
                    + "It is not possible to assign more values than leds."
                    + "Not changing LED status."
                )
                return
        except TypeError as type_err:
            rospy.logwarn(
                "[Piksi Status Indicator]: "
                + "One of the passed arguments is not a list. Not changing LED status."
            )
            return

        for id, stat_val, color in zip(ids, status, rgb_color):
            self.__led_status[id] = stat_val
            if stat_val is not 0:
                self.__led_colors[id] = color
            else:
                # turn off led
                self.__led_colors[id] = [0, 0, 0]
    
    def turn_off_all_leds(self):
        """Turn all LEDs to off"""
        self.__led_status = [0] * self.__num_of_leds
        self.__led_colors = [[0, 0, 0]] * self.__num_of_leds
        self.publish_led_status()
        
    def publish_led_status(self):
        """Publish new status of LEDs.

        This method should only be called if new led status or color has been set.
        """
        msg = StatusLEDArray()

        for status, color in zip(self.__led_status, self.__led_colors):
            led_stat_msg = StatusLED()
            led_stat_msg.red = color[0]
            led_stat_msg.green = color[1]
            led_stat_msg.blue = color[2]
            # Led specific frequency not implemented yet,
            # therefore just blinking or not
            if status > 0:
                led_stat_msg.blinking = True
            else:
                led_stat_msg.blinking = False
            msg.LED_array.append(led_stat_msg)

        self.__msg_pub.publish(msg)

