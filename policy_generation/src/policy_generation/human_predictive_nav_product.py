from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef

from random import choice
import numpy as np
from numpy.random import random_sample

def weighted_values(values, probabilities, size=1):
    values=np.array(values)
    probabilities=np.array(probabilities)
    bins = np.add.accumulate(probabilities)
    return values[np.digitize(random_sample(size), bins)]



class HumanPredictiveNavProduct(Mdp):
    def __init__(self, 
                 states_file, 
                 labels_file,
                 #automaton_file,
                 policy_file,
                 robot_init_x, 
                 robot_init_y,
                 human_init_x,
                 human_init_y,
                 n_rows,
                 n_columns,
                 n_time_steps):
        Mdp.__init__(self)               
        ##list of attributes of an MDP object
        #self.n_state_vars=0 #numer of variables used to defined the mdp state
        #self.state_vars=[] #names of the different variables used to define the state
        #self.state_vars_range={} #ranges for the state vars
        #self.initial_state={} #dict indexed by the state vars
        #self.n_props=0 #number of propositional labels
        #self.props=[] #list of propositional label names
        #self.props_def={} #dict of MdpPropDef instances. keys are the propositional labels names
        #self.n_actions=0 #number of actions
        #self.actions=[] #list of action names
        #self.transitions=[] #list of MdpTransitionDef instances
        #self.current_policy=[]
        #self.reward_names={}
        
        #self.state_vars=['_da', 'robot_x', 'robot_y', 'human_x', 'human_y','time_step']
        self.state_vars_range['robot_x']=[1, n_columns]
        self.state_vars_range['robot_y']=[1, n_rows]
        self.state_vars_range['human_x']=[0, n_columns]
        self.state_vars_range['human_y']=[0, n_rows]
        self.state_vars_range['time_step']=[0, n_time_steps-1]
        self.initial_state['robot_x']=robot_init_x
        self.initial_state['robot_y']=robot_init_y
        self.initial_state['human_x']=human_init_x
        self.initial_state['human_y']=human_init_y
        self.initial_state['time_step']=0
        
        
        self.n_da_states=0
        self.final_automaton_state=0
        self.flat_states = []
        self.n_flat_states = 0
        self.read_states(states_file)
        self.read_transitions(policy_file)
        self.initial_flat_state = self.get_initial_state(labels_file)
        print(self.initial_state)
        self.current_state = self.initial_flat_state
        #self.simulate_random()
        #self.get_current_nav_policy()
        
    def update_state(self, waypoint_val):
        possible_next_states = self.next_states[self.current_state]
        for state in possible_next_states:
            if self.flat_states[state]['waypoint'] == waypoint_val:
                self.current_state = state
                return
        print "Jumped a state!"
        new_state = dict(self.flat_states[self.current_state])
        new_state['waypoint'] = waypoint_val
        for i in range(0, len(self.flat_states)):
            if self.flat_states[i] == new_state: 
                self.current_state = i
                print "Updated MDP state correctly"
                return
        print "Update mdp state error!"
        
      

    def compare_policy_mode(self,state1, state2):
        for key in state1:
            if key != 'waypoint':
                if state1[key]!=state2[key]:
                    return False
        return True
        
    
        
    def simulate_random(self):
        current_state = self.initial_flat_state
        while True:
            action = self.actions[current_state]
            print action
            if action == '':
                return
            current_state = weighted_values(self.next_states[current_state], self.probs[current_state], 1)[0]
            print(self.flat_states[current_state])
            
    def calculate_mean_n_steps(self, n_trials):        
        counter=0.0
        for i in range(0,n_trials):
            current_state = self.initial_flat_state
            while True:
                action = self.actions[current_state]
                if action == '':
                    break
                counter+=1
                current_state = weighted_values(self.next_states[current_state], self.probs[current_state], 1)[0]
        print("MEAN: " + str(counter/n_trials))   
        
    def check_actions(self):
        for i in range(0, self.n_flat_states):
            if self.actions[i] == '':
                if self.flat_states[i]["_da"]==0 and self.flat_states[i]["time_step"]<20:
                    print(self.flat_states[i])      
        
        
    def read_transitions(self, policy_file):
        f = open(policy_file, 'r')
        self.actions = ['' for i in range(0, self.n_flat_states)]
        self.probs = [[] for i in range(0, self.n_flat_states)]
        self.next_states = [[] for i in range(0, self.n_flat_states)]
        f.readline()
        
        for line in f:
            line_array = line.split(' ')
            line_array[-1] = line_array[-1].strip('\n')
            source = int(line_array[0])
            target = int(line_array[1])
            prob = float(line_array[2])
            action = line_array[3]
            self.actions[source] = action
            self.next_states[source].append(target)
            self.probs[source].append(prob)
        f.close()
        
    def get_initial_state(self, labels_file):
        f = open(labels_file, 'r')
        f.readline()
        found = False
        for line in f:
            line_array = line.split(' ')
            if line_array[-1] == '1\n':
                final_state_index = int(line_array[0].strip(':'))
                found = True
                break
        f.close()
        print("FOUND" + str(found))
        if found:
            self.final_automaton_state = self.flat_states[final_state_index]['_da']
        else:
            self.final_automaton_state = 1
        
        if self.final_automaton_state == 0:
            initial_automaton_state = self.n_da_states
        elif self.final_automaton_state == self.n_da_states:
            initial_automaton_state = 0
        else:
            print "ISSUE WITH INITIAL AUTOMATON STATE"
        self.initial_state['_da'] = initial_automaton_state
        print(self.initial_state)
        flat_state = 0
        for state in self.flat_states:
            if state == self.initial_state:
                return flat_state
            flat_state = flat_state + 1
   
        
    def read_states(self, states_file):
        f = open(states_file, 'r')
        
        variables = f.readline()
        variables = variables.split(',')
        variables[0] = variables[0].strip('(')
        variables[-1] = variables[-1].strip(')\n')
        self.n_state_vars = 0
        for variable in variables:
            self.n_state_vars = self.n_state_vars + 1
            self.state_vars.append(variable)

        
        for line in f:
            states_string = line.split(':')
            if int(states_string[0]) != self.n_flat_states:
                print "ERROR READING POLICY STATES"
            flat_state_list = states_string[1].split(',')
            flat_state_list[0] = flat_state_list[0].strip('(')
            flat_state_list[-1] = flat_state_list[-1].strip(')\n')
            flat_state_dict={}
            for (var_name, value) in zip(self.state_vars, flat_state_list):
                flat_state_dict[var_name] = int(value)
                if var_name == '_da':
                    self.n_da_states = max(self.n_da_states, int(value))
            self.flat_states.append(flat_state_dict)
            self.n_flat_states = self.n_flat_states + 1
        self.state_vars_range['_da']=[0, self.n_da_states]
        f.close()
        
        
    def get_object_var_name(self, object_name, waypoints_list, index):
        return object_name + '_at_' + waypoints_list[index]
        

    