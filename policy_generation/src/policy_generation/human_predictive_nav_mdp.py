from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef


class HumanPredictiveNavMdp(Mdp):
    def __init__(self, human_mc_file, robot_init_x, robot_init_y, n_rows, n_columns, n_time_steps):
        Mdp.__init__(self)
        time_steps_model=[None for i in range(0,n_time_steps)]
        prob_out_of_map=[]
        for i in range(0,n_time_steps):
            time_steps_model[i]=[None for j in range(0,n_rows)]
            for j in range(0,n_rows):
                time_steps_model[i][j]=[0 for k in range(0,n_columns)]
        stream=open(human_mc_file,'r')
        row=0
        for line in stream:
            line=line.split(' ')
            for time_step in range(0,n_time_steps):
                for column in range(0,n_columns):
                    time_steps_model[time_step][n_rows-row-1][column]=float(line[time_step*n_columns+column])
            row+=1        
        stream.close()
        
        
        for time_step in time_steps_model:
            total_prob=0
            for row in time_step:
                for value in row:
                    total_prob+=value
            prob_out_of_map.append(round(1-total_prob,3))
        print prob_out_of_map
        
        self.state_vars=['robot_x', 'robot_y', 'human_x', 'human_y','time_step']
        self.state_vars_range['robot_x']=[1, n_columns]
        self.state_vars_range['robot_y']=[1, n_rows]
        self.state_vars_range['human_x']=[0, n_columns]
        self.state_vars_range['human_y']=[0, n_rows]
        self.state_vars_range['time_step']=[0, n_time_steps-1]
        
        initial_time_step=time_steps_model[0]
        found_initial_state=False
        for row in range(0,n_rows):
            for column in range(0,n_columns):
                if initial_time_step[row][column]==1:
                    found_initial_state=True
                    break
            if found_initial_state:
                break
        if found_initial_state:
            self.initial_state['human_x']=column+1
            self.initial_state['human_y']=row+1
        else:
            rospy.logerr("Error finding human initial state")
            return
        
    
        self.initial_state['robot_x']=robot_init_x
        self.initial_state['robot_y']=robot_init_y
        
        self.initial_state['time_step']=0
        self.n_props=1
        self.props=['close_to_human']
        self.props_def['close_to_human']=MdpPropDef(name='close_to_human', 
                                                    conds='(human_x>0) & (robot_x>human_x?robot_x-human_x<2:human_x-robot_x<2) & (robot_y>human_y?robot_y-human_y<2:human_y-robot_y<2)')
        self.n_actions=5
        self.actions=['move_up', 'move_left', 'move_right', 'wait']
        self.reward_names=['time']
        for t in range(0,n_time_steps-1):
            current_human_probs=time_steps_model[t+1]
            human_prob_post_conds=[]
            for row in range(0,n_rows):
                for column in range(0, n_columns):
                    prob = current_human_probs[row][column]
                    if prob > 0:                                            
                        human_prob_post_conds.append([prob, '(human_x\'=' + str(column+1) + ') & (human_y\'=' + str(row+1) + ') & (time_step\'=' + str(t+1) + ')'])
            if prob_out_of_map[t+1] > 0:
                human_prob_post_conds.append([prob_out_of_map[t+1], '(human_x\'=0) & (human_y\'=0) & (time_step\'=' + str(t+1) + ')'])
            action='move_up'
            pre_conds='(robot_y>1) & (time_step=' + str(t) + ')'
            prob_post_conds=[list(element) for element in human_prob_post_conds]
            for post_cond in prob_post_conds:
                post_cond[1]+= ' & ' + '(robot_y\'=robot_y-1)'
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_down'
            pre_conds='(robot_y<' + str(n_rows) + ') & (time_step=' + str(t) + ')'
            prob_post_conds=[list(element) for element in human_prob_post_conds]
            for post_cond in prob_post_conds:
                post_cond[1]+= ' & ' + '(robot_y\'=robot_y+1)'
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_left'
            pre_conds='(robot_x>1) & (time_step=' + str(t) + ')'
            prob_post_conds=[list(element) for element in human_prob_post_conds]
            for post_cond in prob_post_conds:
                post_cond[1]+= ' & ' + '(robot_x\'=robot_x-1)'
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_right'
            pre_conds='(robot_x<' + str(n_columns) + ') & (time_step=' + str(t) + ')'
            prob_post_conds=[list(element) for element in human_prob_post_conds]
            for post_cond in prob_post_conds:
                post_cond[1]+= ' & ' + '(robot_x\'=robot_x+1)'
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='wait'
            pre_conds='(time_step=' + str(t) + ')'
            prob_post_conds=list(human_prob_post_conds)
            rewards={'time':0.99} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
        