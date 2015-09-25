#! /usr/bin/env python
import os
import sys
import rospy

from mdp_plan_exec.prism_java_talker import PrismJavaTalker
from policy_generation.human_predictive_nav_mdp import HumanPredictiveNavMdp



   
class MdpNavServer(object):
    def __init__(self, human_mc_file, n_rows=40, n_columns=9, n_time_steps=40):       
        self.directory = os.path.expanduser("~") + '/tmp/prism/human_predictive_nav/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name="human_nav.mdp"
        #self.prism_policy_generator=PrismJavaTalker(8088,self.directory, self.file_name)
        
        self.human_nav_mdp=HumanPredictiveNavMdp(human_mc_file, n_rows, n_columns, n_time_steps)
        self.human_nav_mdp.write_prism_model('/home/bruno/Desktop/test.mdp')

        
        #self.mdp_nav_as=SimpleActionServer('object_search_mdp', ObjectSearchMdpAction, execute_cb = self.execute_policy_cb, auto_start = False)
        #self.mdp_nav_as.register_preempt_callback(self.preempt_policy_execution_cb)
        #self.mdp_nav_as.start()
        
        rospy.loginfo("MDP  human nav initialised.")

     
    #def main(self):
        ## Wait for control-c
        #rospy.spin()       
        #if rospy.is_shutdown():
            #self.prism_policy_generator.shutdown(False)

if __name__ == '__main__':
    rospy.init_node('mdp_human_nav')
    mdp_nav =  MdpNavServer(sys.argv[1])
    #mdp_nav.main()
    
    
