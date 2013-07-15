'''
Client class for switching between light show stacks.

2011.4.13 Peter Pastor
'''

import rospy
from dec_light_show_msgs.srv import SwitchLightShowStack, SwitchLightShowStackResponse

class LightShowStackSwitcher:
    def __init__(self):
        ''' Client class for switching light show stacks. '''
        self.switch_light_show_service_name = '/DecLightShowManager/switchLightShowStack'
        rospy.wait_for_service(self.switch_light_show_service_name)
        self.switch_light_show_service = rospy.ServiceProxy(self.switch_light_show_service_name, SwitchLightShowStack)
        self.current_light_show_stack = []

    def list_stacks(self):
        ''' List stacks. Returns True on success and False on failure. '''
        light_show_stacks = []
        response = SwitchLightShowStackResponse()
        try:
            response = self.switch_light_show_service(light_show_stacks)
        except rospy.ServiceException, e:
            print "Service %s did not process request: %s " % (self.switch_light_show_service_name, str(e))
        if response.result != SwitchLightShowStackResponse.SUCCEEDED:
            return False

        for i, stack_name in enumerate(response.light_show_stacks):
            print "%i %s" %(i, stack_name)

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
