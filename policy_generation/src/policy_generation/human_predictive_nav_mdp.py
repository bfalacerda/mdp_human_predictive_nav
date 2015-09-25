from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef


class HumanPredictiveNavMdp(Mdp):
    def __init__(self, human_mc_file, n_rows, n_columns, n_time_steps):
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
        self.state_vars_range['robot_x']=[1, n_rows]
        self.state_vars_range['robot_y']=[1, n_columns]
        self.state_vars_range['human_x']=[0, n_rows]
        self.state_vars_range['human_y']=[0, n_columns]
        self.state_vars_range['time_step']=[0, n_time_steps-1]
        
        self.initial_state['robot_x']=5
        self.initial_state['robot_y']=1
        self.initial_state['human_x']=9
        self.initial_state['human_y']=13
        self.initial_state['time_step']=0
        self.n_props=1
        self.props=['close_to_human']
        self.props_def['close_to_human']=MdpPropDef(name='close_to_human',
                                                    conds='(abs(robot_x - human_x) < 2) & (abs(robot_y - human_y) < 2)')
        self.n_actions=5
        self.actions=['move_up', 'move_left', 'move_right', 'wait']
        self.reward_names=['time']
        for t in range(0,n_time_steps):
            action='move_up'
            pre_conds='(robot_y > 1) & (time_step = ' + str(t) + ')'
            prob_post_conds=[[0.5, '(robot_y=robot_y+1) & (human_x=3) & (human_y=4) & (time_step = ' + str(t+1) + ')'], 
                              [0.5,'(robot_y=robot_y+1) & (human_x=4) & (human_y=10) & (time_step = ' + str(t+1) + ')']]
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_down'
            pre_conds='(robot_y > 1) & (time_step = ' + str(t) + ')'
            prob_post_conds=[[0.5, '(robot_y=robot_y+1) & (human_x=3) & (human_y=4) & (time_step = ' + str(t+1) + ')'], 
                              [0.5,'(robot_y=robot_y+1) & (human_x=4) & (human_y=10) & (time_step = ' + str(t+1) + ')']]
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_left'
            pre_conds='(robot_y > 1) & (time_step = ' + str(t) + ')'
            prob_post_conds=[[0.5, '(robot_y=robot_y+1) & (human_x=3) & (human_y=4) & (time_step = ' + str(t+1) + ')'], 
                              [0.5,'(robot_y=robot_y+1) & (human_x=4) & (human_y=10) & (time_step = ' + str(t+1) + ')']]
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='move_right'
            pre_conds='(robot_y > 1) & (time_step = ' + str(t) + ')'
            prob_post_conds=[[0.5, '(robot_y=robot_y+1) & (human_x=3) & (human_y=4) & (time_step = ' + str(t+1) + ')'], 
                              [0.5,'(robot_y=robot_y+1) & (human_x=4) & (human_y=10) & (time_step = ' + str(t+1) + ')']]
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            action='wait'
            pre_conds='(robot_y > 1) & (time_step = ' + str(t) + ')'
            prob_post_conds=[[0.5, '(robot_y=robot_y+1) & (human_x=3) & (human_y=4) & (time_step = ' + str(t+1) + ')'], 
                              [0.5,'(robot_y=robot_y+1) & (human_x=4) & (human_y=10) & (time_step = ' + str(t+1) + ')']]
            rewards={'time':1} 
            self.transitions.append(MdpTransitionDef(action_name=action,
                                            pre_conds=pre_conds,
                                            prob_post_conds=prob_post_conds,
                                            rewards=rewards,
                                            exec_count=0))
            