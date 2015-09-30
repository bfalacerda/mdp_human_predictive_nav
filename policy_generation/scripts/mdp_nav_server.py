#! /usr/bin/env python
import os
import sys
import rospy

from object_search_mdp.prism_java_talker import PrismJavaTalker
from policy_generation.human_predictive_nav_mdp import HumanPredictiveNavMdp
from policy_generation.human_predictive_nav_product import HumanPredictiveNavProduct


   
class MdpNavServer(object):
    def __init__(self, human_mc_file, robot_init_x, robot_init_y, n_rows=40, n_columns=9, n_time_steps=40):       
        
        self.human_nav_mdp=HumanPredictiveNavMdp(human_mc_file, robot_init_x, robot_init_y, n_rows, n_columns, n_time_steps)
        self.human_nav_mdp.write_prism_model('/home/bruno/Desktop/human_test.mdp')
        
   
        self.directory = os.path.expanduser("~") + '/tmp/prism/human_predictive_nav/'
        try:
            os.makedirs(self.directory)
        except OSError as ex:
            print 'error creating PRISM directory:',  ex
        self.file_name="human_nav.mdp"
        self.prism_policy_generator=PrismJavaTalker(8088,self.directory, self.file_name)
        
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
    mdp_nav =  MdpNavServer(sys.argv[1], sys.argv[2], sys.argv[3])
    mdp_nav.human_nav_mdp.write_prism_model(mdp_nav.directory+mdp_nav.file_name)
    rospy.loginfo("Creating high level search policy...")
    #ltl_spec = 'Pmax=? [  F ("close_to_human")  ]'
    #ltl_spec = 'ijcai(Pmax=? [ (F ((robot_x=2) & (robot_y=20))) ])' #output2.txt
    #ltl_spec = 'ijcai(Pmax=? [ F ("close_to_human") ])' #output1.txt
    ltl_spec = 'ijcai(Pmax=? [ ((!"close_to_human") U ((robot_x=2) & (robot_y=20))) ])' #output3.txt
    expected_time=float(mdp_nav.prism_policy_generator.get_policy(ltl_spec))
    mdp_nav.prism_policy_generator.shutdown(False)
    rospy.loginfo("Policy created")
    product = HumanPredictiveNavProduct(mdp_nav.directory + '/prod.sta', 
                                            mdp_nav.directory + '/prod.lab', 
                                            mdp_nav.directory + '/adv.tra',
                                            int(sys.argv[2]),
                                            int(sys.argv[3]),
                                            mdp_nav.human_nav_mdp.initial_state['human_x'],
                                            mdp_nav.human_nav_mdp.initial_state['human_y'],
                                            40,
                                            9,
                                            40)
    product.write_prism_model("/home/bruno/Desktop/product.mdp")
    for i in range(0,10):
        product.simulate_random()
        print('-----------------\n\n')
    product.calculate_mean_n_steps(10000)
    #product.check_actions()
    
