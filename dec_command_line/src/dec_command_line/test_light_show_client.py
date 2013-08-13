'''
Client class for test light show.

2011.4.13 Peter Pastor
'''
import os
import rospy
from dec_msgs.srv import Test
from dec_msgs.msg import LightShowFrame

class TestLightShowClient:
    def __init__(self):
        ''' Client class for test light show. '''
        self.test_light_show_service_name = '/DecLightShowManager/test'
        rospy.wait_for_service(self.test_light_show_service_name)
        self.test_light_show_service = rospy.ServiceProxy(self.test_light_show_service_name, Test)

    def test(self):
        os.system('clear')
        # get current frame
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
        for option_description, i in enumerate(options):
            print "[%i] %s" (i, option_description)
        
        self.getInput(prompt = "Enter option:", min = 0, max = len(options))        

        if option == 0:
            print "Enter node id between 0 and %i." %(len(current_light_show_frame.block_node_levels))
            

    def getInput(self, prompt, min, max):
        done = False
        while not done:
            s = raw_input(prompt)
            input = 0
            try:
                input = int(s)
            except ValueError:            
                print "ERROR: invalid input >%s<. Enter number between [%i..%i]." %(min, max)
                continue
            done = True

    def testLightShow(self, test_light_show_frame):
        request = TestRequest()
        request.frame = test_light_show_frame
        response = TestResponse()
        try:
            response = self.test_light_show_service(request)
        except rospy.ServiceException, e:
            print "Service %s did not process request: %s " % (self.test_light_show_service, str(e))
        if response.result != TestResponse.SUCCEEDED:
            return False
        return True, response.frame
