'''
Client class for switching between light show stacks.

2011.4.13 Peter Pastor
'''

import rospy
from dec_light_show_msgs.srv import SwitchLightShowStack, SwitchLightShowStackResponse
from dec_msgs.msg import Brightness

class LightShowStackSwitcher:
    def __init__(self):
        ''' Client class for switching light show stacks. '''
        self.switch_light_show_service_name = '/DecLightShowManager/switchLightShowStack'
        rospy.wait_for_service(self.switch_light_show_service_name)
        self.switch_light_show_service = rospy.ServiceProxy(self.switch_light_show_service_name, SwitchLightShowStack)
        [light_show_stacks, active_light_show_stacks], result = self.get_stacks() 
        self.current_light_show_stack = light_show_stacks
        self.current_active_light_show_stack = active_light_show_stacks
        self.brightness_publisher = rospy.Publisher('//DecLightShowManager/BrightnessProcessor/command', Brightness)

    def get_stacks(self):
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
        brightness_set = False
        while not brightness_set:
            s = raw_input("Enter brightness:")
            brightness = 0
            try:
                brightness = int(s)
            except ValueError:            
                print "ERROR: invalid brightness >%s<. Enter number between [0..255]." % s
                continue
        
            bmsg = Brightness()
            bmsg.node_brightness = brightness
            bmsg.beam_brightness = brightness
            self.brightness_publisher.publish(bmsg)
            brightness_set = True        

    def set(self, id):
        if id < 0 or id >= len(self.current_light_show_stack):
            self.list_stacks()
            print "Invalid id >%i< must be within [0..%i]." % (id, len(self.current_light_show_stack) - 1)
            return False
        
        if self.current_light_show_stack[id] == "BrightnessProcessorStack":
            self.setBrightness()

        print "Switching to stack >%s<." % self.current_light_show_stack[id]
        if self.switch_light_show_stack(self.current_light_show_stack[id]):
            self.list_stacks()
            return True
        return False

    def list_stacks(self):
        [light_show_stacks, active_light_show_stacks], result = self.get_stacks() 
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
        
    def switch_light_show_stack(self, light_show_stack):
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
