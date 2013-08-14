'''
Client class for light show.

2011.4.13 Peter Pastor
'''

import rospy
import os

from dec_msgs.srv import Test, TestRequest, TestResponse
from dec_msgs.msg import LightShowFrame

from dec_light_show_msgs.srv import SwitchLightShowStack, SwitchLightShowStackResponse
from dec_msgs.msg import Brightness

class LightShowClient:
    def __init__(self):
        ''' Client class for light show. '''
        self.switch_light_show_service_name = '/DecLightShowManager/switchLightShowStack'
        rospy.wait_for_service(self.switch_light_show_service_name)
        self.switch_light_show_service = rospy.ServiceProxy(self.switch_light_show_service_name, SwitchLightShowStack)
        [light_show_stacks, active_light_show_stacks], result = self.getStacks() 
        self.current_light_show_stack = light_show_stacks
        self.current_active_light_show_stack = active_light_show_stacks
        self.brightness_publisher = rospy.Publisher('/DecLightShowManager/BrightnessProcessor/command', Brightness)
        self.test_light_show_service_name = '/DecLightShowManager/test'
        rospy.wait_for_service(self.test_light_show_service_name)
        self.test_light_show_service = rospy.ServiceProxy(self.test_light_show_service_name, Test)

    def test(self):
        os.system('clear')
        result, current_light_show_frame = self.testLightShow(test_light_show_frame = LightShowFrame())
        if not result:
            print "Problem... try again maybe?"
            return True
        os.system('clear')
        print "Options:"
        options = []
        options.append("set light level for a single node")
        options.append("set light level for range of nodes")
        options.append("set light level for all nodes")
        for i, option_description in enumerate(options):
            print "[%i] %s" % (i, option_description)
        
        option = self.getInput(prompt = "Enter option", min = 0, max = len(options), default = 0)        

        if option == 0:
            min = 0
            max = len(current_light_show_frame.block_node_levels)
            node_id = self.getInput("Enter node id between " + str(min) + " and " + str(max) + "", min, max + 1, default = 0)
            max = 100
            node_level = self.getInput("Enter node level between " + str(min) + " and " + str(max) + "", min, max + 1, default = 100)
            list_current = list(current_light_show_frame.block_node_levels)
            list_current[node_id] = node_level / 100.0 
            current_light_show_frame.block_node_levels = tuple(list_current)
            result, frame = self.testLightShow(current_light_show_frame)
            if not result:
                print "Problem when setting node level."
            os.system('clear')
        if option == 1:
            min = 0
            max = len(current_light_show_frame.block_node_levels) - 1
            start_node_id = self.getInput("Enter start node id between " + str(min) + " and " + str(max) + "", min, max + 1, default = 0)
            min = start_node_id
            max = len(current_light_show_frame.block_node_levels)
            end_node_id = self.getInput("Enter end node id between " + str(min) + " and " + str(max) + "", min, max + 1, default = max)
            min = 0
            max = 100
            node_level = self.getInput("Enter node level between " + str(min) + " and " + str(max) + "", min, max + 1, default = 100)
            list_current = list(current_light_show_frame.block_node_levels)
            for i in range(start_node_id, end_node_id):
                list_current[i] = node_level / 100.0 
            current_light_show_frame.block_node_levels = tuple(list_current)
            result, frame = self.testLightShow(current_light_show_frame)
            if not result:
                print "Problem when setting node level."
            os.system('clear')
        if option == 2:
            min = 0
            max = 100
            node_level = self.getInput("Enter node level between " + str(min) + " and " + str(max) + "", min, max + 1, default = 100)
            list_current = list(current_light_show_frame.block_node_levels)
            for i in range(0, len(current_light_show_frame.block_node_levels)):
                list_current[i] = node_level / 100.0 
            current_light_show_frame.block_node_levels = tuple(list_current)
            result, frame = self.testLightShow(current_light_show_frame)
            if not result:
                print "Problem when setting node level."
            os.system('clear')

    def getInput(self, prompt, min, max, default):
        done = False
        input = 0
        while not done:
            s = raw_input(prompt + " [" + str(default) + "]:")            
            try:
                if s == "":
                    input = default
                    done = True
                    continue
                input = int(s)
                if input >= min and input < max:
                    done = True
                else:
                    print "Input out of range [%i..%i]." %(min, max-1)
            except ValueError:
                print "ERROR: invalid input >%s<. Enter number between [%i..%i]." %(s, min, max-1)
                continue
        return input

    def testLightShow(self, test_light_show_frame):
        request = TestRequest()
        request.frame = test_light_show_frame
        response = TestResponse()
        try:
            response = self.test_light_show_service(request)
        except rospy.ServiceException, e:
            print "Service %s did not process request: %s " % (self.test_light_show_service, str(e))
        if response.result != TestResponse.SUCCEEDED:
            return False, None
        return True, response.frame

    def getStacks(self):
        ''' List stacks. Returns True on success and False on failure. '''
        light_show_stacks = []
        response = SwitchLightShowStackResponse()
        try:
            response = self.switch_light_show_service(light_show_stacks)
        except rospy.ServiceException, e:
            print "Service %s did not process request: %s " % (self.switch_light_show_service_name, str(e))
        if response.result != SwitchLightShowStackResponse.SUCCEEDED:
            return None, False
        return [response.light_show_stacks, response.active_light_show_stacks], True

    def setBrightness(self):
        node_brightness_set = False
        beam_brightness_set = False
        while not node_brightness_set and not beam_brightness_set:
            if not node_brightness_set:
                s = raw_input("Enter node brightness:")
                node_brightness = 0
                try:
                    node_brightness = int(s)
                except ValueError:            
                    print "ERROR: invalid node brightness >%s<. Enter number between [0..255]." % s
                    continue
                node_brightness_set = True        
        
            if not beam_brightness_set:
                s = raw_input("Enter beam brightness:")
                beam_brightness = 0
                try:
                    beam_brightness = int(s)
                except ValueError:            
                    print "ERROR: invalid beam brightness >%s<. Enter number between [0..255]." % s
                    continue
                beam_brightness_set = True        
        
            bmsg = Brightness()
            bmsg.node_brightness = node_brightness
            bmsg.beam_brightness = beam_brightness
            self.brightness_publisher.publish(bmsg)

    def set(self, id):
        if id < 0 or id >= len(self.current_light_show_stack):
            self.listStacks()
            print "Invalid id >%i< must be within [0..%i]." % (id, len(self.current_light_show_stack) - 1)
            return False
        
        if self.current_light_show_stack[id] == "BrightnessProcessorStack":
            self.setBrightness()

        print "Switching to stack >%s<." % self.current_light_show_stack[id]
        if self.switchLightShowStack(self.current_light_show_stack[id]):
            if self.current_light_show_stack[id] == "TestLightGeneratorStack":
                self.test()            
            self.listStacks()
            return True
        return False

    def listStacks(self):
        [light_show_stacks, active_light_show_stacks], result = self.getStacks() 
        if not result:
            return False

        for i, stack_name in enumerate(light_show_stacks):
            active = False
            for j, active_stack_name in enumerate(active_light_show_stacks):
                if stack_name == active_stack_name:
                    active = True
            if active:        
                print "%i\t [X] %s" %(i, stack_name)
            else:
                print "%i\t [ ] %s" %(i, stack_name)
        return True
        
    def switchLightShowStack(self, light_show_stack):
        ''' Switch the light show stack. Returns True on success and False on failure. '''
        light_show_stacks = [light_show_stack]
        response = SwitchLightShowStackResponse()
        try:
            response = self.switch_light_show_service(light_show_stacks)
        except rospy.ServiceException, e:
            print "Service %s did not process request: %s " % (self.switch_light_show_service_name, str(e))
        if response.result != SwitchLightShowStackResponse.SUCCEEDED:
            return False
        return True
