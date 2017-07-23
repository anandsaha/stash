from robot import RobotArm
import numpy as np

class RobotArmAgent(object):

    def __init__(self, robot, alpha=0.1, gamma=0.9, epsilon=0.2, q_init=1):
    
        self.robot = robot
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon

        self.tolerance = 0.01
            
        # The actions the agent can take - either goto some x,y,z position 
        # or engage/disengage claw
        dim = self.robot.get_env_dimensions()
        x_range = np.arange(dim[0][0], dim[0][1], self.tolerance)
        y_range = np.arange(dim[1][0], dim[1][1], self.tolerance)
        z_range = np.arange(dim[2][0], dim[2][1], self.tolerance)

        self.action_type1 = 'move_position'
        self.action_type2 = 'engage_claw'
        self.actions = []

        for x in x_range:
            for y in y_range:
                for z in z_range:
                    self.actions.append([self.action_type1, [x, y, z]])
        
        self.actions.append([self.action_type2, True])
        self.actions.append([self.action_type2, False])
        self.total_actions = len(self.actions) 

        # The states the environment can be in, consists of:
        # a. Position of the cylinder

        self.states = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    self.states.append([x, y, z])
        self.total_states = len(self.states)

        print("There are {0} actions.".format(self.total_actions))
        print("There are {0} states.".format(self.total_states))

        self.q_table = np.full((self.total_states, self.total_actions), q_init)

    def init(self):
        self.robot.restart_sim()
        self.state_id = self.observe_state()
        self.claw_position = self.robot.get_position(self.robot.claw_handle) 

        # The bin should be static, it's location should not change
        self.bin_position = self.robot.get_position(self.robot.bin_handle)
        self.bin_top_position = self.bin_position # This location is on top of the bin
        self.bin_top_position[2] *= 3 # This location is on top of the bin
        # Cylinder height should be constant, else it has fallen
        self.cylinder_height = self.robot.get_position(self.robot.cylinder_handle)[2]


    def choose_action(self, state_id):
        if np.random.uniform() < self.epsilon:
             action_id = np.random.choice(self.total_actions)
        else:
             action_id = np.argmax(self.q_table[state_id])
        return action_id


    def do_action(self, action_id):
        action = self.actions[action_id]
        if action[0] == self.action_type1:
            print('Moving to position: ', action[1])
            self.robot.goto_position(action[1])
        elif action[0] == self.action_type2:
            print('Enabling claw: ', action[1])
            self.robot.enable_claw(action[1])


    def distance(self, pos1, pos2):
        x2 = np.square(pos1[0] - pos2[0])
        y2 = np.square(pos1[1] - pos2[1])
        z2 = np.square(pos1[2] - pos2[2])

        return np.sqrt(x2 + y2 + z2)


    def play(self):

        pre_distance = self.distance(self.states[self.state_id], self.claw_position)

        action_id = self.choose_action(self.state_id)
        self.do_action(action_id)

        try:
            state_id_new = self.observe_state()
        except RuntimeError as err:
            print(err)
            return True

        state_new = self.states[state_id_new]
        claw_position_new = self.robot.get_position(self.robot.claw_handle)
        bin_position_new = self.robot.get_position(self.robot.bin_handle)

        terminate = False
        reward = 0
        if np.abs(state_new[2] - self.cylinder_height) > self.tolerance:
            reward = -1 # Cylinder has fallen
            print('Cylinder has fallen, terminating')
            terminate = True
        elif self.distance(self.bin_position, bin_position_new) > 0.1:
            print(self.distance(self.bin_position, bin_position_new))
            reward = -1 # Bin was shifted
            print('Bin has shifted, terminiting')
            terminate = True
        else:
            post_distance = self.distance(state_new, claw_position_new)
            forward = pre_distance - post_distance
            reward = forward 

        # update Q-table
        self.update_q(self.state_id, action_id, reward, state_id_new)

        self.state_id = state_id_new
        self.claw_position = claw_position_new

        return terminate


    def update_q(self, state, action, reward, state_new):
        q_sa = self.q_table[state, action]
        td_error = reward + self.gamma * np.max(self.q_table[state_new]) - q_sa
        self.q_table[state, action] = q_sa + self.alpha * td_error


    def observe_state(self):
        pos = self.robot.get_position(self.robot.cylinder_handle)
        print('State is at ', pos)

        state_id = 0
        for state in self.states:
            if abs(state[0] - pos[0]) <= self.tolerance and abs(state[1] - pos[1]) <= self.tolerance and abs(state[2] - pos[2]) <= self.tolerance:
                break
            state_id += 1

        if state_id == self.total_states:
            raise RuntimeError("Bad state_id detected")  

        return state_id


ra = RobotArm('127.0.0.1', 19997)
raa = RobotArmAgent(ra)

episodes = 100

while episodes > 0:
    print ('Episode ', episodes)
    raa.init()
    i = 0
    while i < 100:
        terminate = raa.play()
        if terminate == True:
            break
        i += 1

    episodes -= 1

 
