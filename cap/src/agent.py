from robot import RobotArm
import numpy as np
import time


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

        self.action_type1 = 'move_claw'
        self.action_type2 = 'engage_claw'
        self.actions = []

        self.actions.append([self.action_type2, True])
        self.actions.append([self.action_type2, False])

        for x in x_range:
            for y in y_range:
                for z in z_range:
                    self.actions.append([self.action_type1, [x, y, z]])

        self.total_actions = len(self.actions)

        # The states the environment can be in, consists of:
        # a. Position of the cylinder (the item to pick)

        self.states = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    self.states.append([x, y, z])
        self.total_states = len(self.states)
        self.states.append([-100, -100, -100])  # This is to satisfy the condition when state_id is not found

        print("There are {0} actions.".format(self.total_actions))
        print("There are {0} states.".format(self.total_states))

        self.q_table = np.full((self.total_states, self.total_actions), q_init)

        # Give the claw engaging actions high initial values
        for index, val in enumerate(self.q_table):
            self.q_table[index][0] = 2
            self.q_table[index][1] = 2

    def init(self):
        self.robot.restart_sim()

        self.state_id = self.observe_state()
        # self.claw_position = self.robot.get_position(self.robot.claw_handle)

        # The bin should be static, it's location should not change
        self.bin_position = self.robot.get_position(self.robot.bin_handle)
        self.bin_top_position = list(self.bin_position)  # This location is on top of the bin
        self.bin_top_position[2] *= 3  # This location is on top of the bin
        # Cylinder height should be constant, else it has fallen
        self.cylinder_height = self.robot.get_position(self.robot.cylinder_handle)[2]

        self.claw_engaged = False
        self.action_type = self.action_type1

        self.actionstate_prev = {}
        self.actionstate_curr = {}

    def get_cannonical_position(self, handle):
        pos = self.robot.get_position(handle)

        state_id = 0
        for state in self.states:
            if abs(state[0] - pos[0]) <= self.tolerance and abs(state[1] - pos[1]) <= self.tolerance and abs(
                            state[2] - pos[2]) <= self.tolerance:
                break
            state_id += 1

        return state_id

    def update_actionstate(self, action_id):

        self.actionstate_curr['action_id'] = action_id
        self.actionstate_curr['state_id'] = self.observe_state()
        self.actionstate_curr['cylinder_position'] = self.robot.get_position(self.robot.cylinder_handle)
        self.actionstate_curr['bin_position'] = self.robot.get_position(self.robot.bin_handle)
        self.actionstate_curr['claw_position'] = self.robot.get_position(self.robot.claw_handle)

        self.actionstate_curr['claw_cylinder_distance'] = self.distance(self.actionstate_curr['claw_position'],
                                                                        self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['bin_cylinder_distance'] = self.distance(self.actionstate_curr['bin_position'],
                                                                       self.actionstate_curr['cylinder_position'])

        # Is the cylinder inside the bin?
        cylinder_bin_distance = self.distance(self.actionstate_curr['cylinder_position'],
                                              self.actionstate_curr['bin_position'])
        self.actionstate_curr['cylinder_in_bin'] = False
        if cylinder_bin_distance < 0.05:
            self.actionstate_curr['cylinder_in_bin'] = True

    def choose_action(self, state_id):
        if np.random.uniform() < self.epsilon:
            action_id = np.random.choice(self.total_actions)
        else:
            action_id = np.argmax(self.q_table[state_id])
        return action_id

    def do_action(self, action_id):

        action = self.actions[action_id]
        self.action_type = action[0]

        if action[0] == self.action_type1:
            print('Action: Moving claw')
            self.robot.goto_position(action[1])
        elif action[0] == self.action_type2:
            print('Action: Engaging/Disengaging claw')
            self.robot.enable_claw(action[1])
            self.claw_engaged = action[1]

    @staticmethod
    def distance(pos1, pos2):
        x2 = np.square(pos1[0] - pos2[0])
        y2 = np.square(pos1[1] - pos2[1])
        z2 = np.square(pos1[2] - pos2[2])

        return np.sqrt(x2 + y2 + z2)

    def update_q(self, state, action, reward, state_new):
        q_sa = self.q_table[state, action]
        td_error = reward + self.gamma * np.max(self.q_table[state_new]) - q_sa
        self.q_table[state, action] = q_sa + self.alpha * td_error

    def observe_state(self):
        return self.get_cannonical_position(self.robot.cylinder_handle)

    def calculate_reward(self, state_id_new, state_new):

        terminate = False
        reward = 0
        if state_id_new == self.total_states:
            reward = -100
            print('Reached invalid state')
            terminate = True
        elif (state_new[2] + 0.01 - self.cylinder_height) < 0:
            reward = -10
            print('Cylinder has fallen, terminating')
            terminate = True
        elif self.distance(self.bin_position, self.actionstate_curr['bin_position']) > 0.01:
            print(self.bin_position)
            print(self.actionstate_curr['bin_position'])
            reward = -10
            print('Bin has shifted, terminiting')
            terminate = True
        else:
            # post_distance = self.distance(state_new, claw_position_new)
            # forward = pre_distance - post_distance
            d1 = self.distance(state_new, self.actionstate_curr['bin_position'])
            d2 = self.distance(state_new, self.actionstate_curr['claw_position'])
            reward = 5 - (d1 + d2)

        return reward, terminate

    def step_through(self):
        self.actionstate_prev = self.actionstate_curr
        action_id = self.choose_action(self.state_id)
        self.do_action(action_id)
        self.update_actionstate(action_id)
        state_id_new = self.actionstate_curr['state_id']
        state_new = self.states[state_id_new]

        reward, terminate = self.calculate_reward(state_id_new, state_new)

        # I was in this state, I took this action, I got this reward, and I reached the new state
        self.update_q(self.state_id, action_id, reward, state_id_new)
        self.state_id = state_id_new
        return terminate


ra = RobotArm('127.0.0.1', 19997)
raa = RobotArmAgent(ra)

episodes = 100

while episodes > 0:
    print ('=============================================> Episode ', episodes)
    raa.init()
    i = 0
    while i < 100:
        terminate = raa.step_through()
        if terminate == True:
            break
        i += 1

    episodes -= 1
