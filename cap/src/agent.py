from robot import RobotArm
import numpy as np
import copy
import os


class RobotArmAgent(object):
    def __init__(self, robot, alpha=0.1, gamma=0.9, epsilon=0.2, q_init=1):

        self.robot = robot
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon

        self.tolerance = 0.005

        # The actions the agent can take - either goto some x,y,z position 
        # or engage/disengage claw
        dim = self.robot.get_env_dimensions()
        x_range = np.arange(dim[0][0], dim[0][1], self.tolerance)
        y_range = np.arange(dim[1][0], dim[1][1], self.tolerance)
        z_range = np.arange(dim[2][0], dim[2][1], self.tolerance)

        self.action_type1 = 'move_claw'
        self.action_type2 = 'engage_claw'
        self.invalid_state = [-100, -100, -100, False]
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
                    for b in [True, False]:
                        self.states.append([x, y, z, b])

        self.states.append(self.invalid_state)  # invalid state, the last state.
        self.total_states = len(self.states)

        print("There are {0} actions.".format(self.total_actions))
        print("There are {0} states.".format(self.total_states))

        self.q_table = np.full((self.total_states, self.total_actions), q_init)

        # Give the claw engaging actions high initial values
        for index, val in enumerate(self.q_table):
            self.q_table[index][0] = 2
            self.q_table[index][1] = 2

        self.state_id = None
        self.actionstate_prev = {}
        self.actionstate_curr = {}

    def save_qtable(self):
        print('Saving Q Table')
        f = 'qtables/qtable.txt.npy'
        try:
            os.remove(f)
        except:
            pass
        np.save(f, self.q_table)

    def init(self):
        self.robot.restart_sim()
        self.state_id = self.observe_state()
        self.actionstate_prev = {}
        self.actionstate_curr = {}

    def get_canonical_position(self, handle, claw_enabled):
        pos = self.robot.get_position(handle)

        state_id = 0
        for state in self.states:
            if abs(state[0] - pos[0]) <= self.tolerance \
                    and abs(state[1] - pos[1]) <= self.tolerance \
                    and abs(state[2] - pos[2]) <= self.tolerance \
                    and state[3] == claw_enabled:
                return state_id
            state_id += 1

        return self.total_states - 1  # The last state, which is the invalid state, will get returned

    def update_actionstate(self, action_id):
        self.actionstate_curr['action_id'] = action_id
        self.actionstate_curr['state_id'] = self.observe_state()
        self.actionstate_curr['state'] = self.states[self.actionstate_curr['state_id']]
        self.actionstate_curr['cylinder_position'] = self.robot.get_position(self.robot.cylinder_handle)
        self.actionstate_curr['bin_position'] = self.robot.get_position(self.robot.bin_handle)
        self.actionstate_curr['claw_position'] = self.robot.get_position(self.robot.claw_handle)

        self.actionstate_curr['claw_cylinder_distance'] = self.distance(self.actionstate_curr['claw_position'],
                                                                        self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['bin_cylinder_distance'] = self.distance(self.actionstate_curr['bin_position'],
                                                                       self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['cylinder_in_bin'] = self.robot.is_object_in_bin()
        self.actionstate_curr['is_cylinder_held'] = self.robot.is_object_held()

    def choose_action(self, state_id):
        if np.random.uniform() < self.epsilon:
            action_id = np.random.choice(self.total_actions)
        else:
            action_id = np.argmax(self.q_table[state_id])
        return action_id

    def do_action(self, action_id):
        action = self.actions[action_id]

        if action[0] == self.action_type1:
            print('Action: Moving claw')
            self.robot.goto_position(action[1])
        elif action[0] == self.action_type2:
            print('Action: Engaging/Disengaging claw')
            self.robot.enable_claw(action[1])

    @staticmethod
    def distance(pos1, pos2):
        x2 = np.square(pos1[0] - pos2[0])
        y2 = np.square(pos1[1] - pos2[1])
        z2 = np.square(pos1[2] - pos2[2])

        return np.sqrt(x2 + y2 + z2)

    def update_q(self, state, action, reward, state_new):
        print(state, action, reward, state_new)
        q_sa = self.q_table[state, action]
        td_error = reward + self.gamma * np.max(self.q_table[state_new]) - q_sa
        self.q_table[state, action] = q_sa + self.alpha * td_error

    def observe_state(self):
        return self.get_canonical_position(self.robot.center_handle, self.robot.claw_enabled)

    def calculate_reward(self):

        state_new = self.actionstate_curr['state']

        terminate = False
        if state_new == self.invalid_state:
            reward = -10
            print('Penalty: Reached invalid state, terminating')
            terminate = True
        elif (ra.get_position(ra.cylinder_handle)[2] - ra.cylinder_z_locus) < -0.005:
            reward = -10
            print('Penalty: Cylinder has fallen, terminating')
            terminate = True
        elif self.distance(ra.bin_position, self.actionstate_curr['bin_position']) > 0.005:
            reward = -10
            print('Penalty: Bin has shifted, terminating')
            terminate = True
        elif self.robot.claw_enabled \
                and not self.actionstate_curr['is_cylinder_held'] \
                and not self.actionstate_curr['cylinder_in_bin']:
            reward = -1
            print('Penalty: Claw is engaged but cylinder is not in claw')
        elif len(self.actionstate_prev) > 0 and self.actionstate_curr['state'] == self.actionstate_prev['state']:
            reward = -1
            print('Penalty: Previous and current state is same')
        elif self.robot.claw_enabled \
                and self.actionstate_curr['is_cylinder_held'] \
                and not self.actionstate_curr['cylinder_in_bin']:
            if (ra.get_position(ra.cylinder_handle)[2] - ra.cylinder_z_locus) > 0.001:
                reward = 10
                print('Reward: Claw could grab and lift the cylinder !!!!!!!!!!!!!!!!!!!!!!!!!')
            else:
                reward = 5
                print('Reward: Claw could grab the cylinder !!!!!!!!!!!!!!!!!!!!!!!!!')
        elif not self.robot.claw_enabled \
                and not self.actionstate_curr['is_cylinder_held'] \
                and self.actionstate_curr['cylinder_in_bin']:
            reward = 100
            print('Reward: Cylinder in bucket !!!!!!!!!!!!!!!!!!!!!!!!!')
            terminate = True
        else:
            # post_distance = self.distance(state_new, claw_position_new)
            # forward = pre_distance - post_distance
            d1 = self.distance(state_new, self.actionstate_curr['bin_position'])
            d2 = self.distance(state_new, self.actionstate_curr['claw_position'])
            reward = 5 - (d1 + d2)
            reward = -1

        return reward, terminate

    def step_through(self):
        self.actionstate_prev = copy.deepcopy(self.actionstate_curr)
        action_id = self.choose_action(self.state_id)
        self.do_action(action_id)
        self.update_actionstate(action_id)
        state_id_new = self.actionstate_curr['state_id']

        reward, terminate = self.calculate_reward()

        # I was in this state, I took this action, I got this reward, and I reached the new state
        self.update_q(self.state_id, action_id, reward, state_id_new)
        self.state_id = state_id_new
        return terminate


ra = RobotArm('127.0.0.1', 19997)
raa = RobotArmAgent(ra)


"""
raa.init()
state = raa.states[raa.observe_state()]
print(state[0])
print(state[1])
print(state[2])
print(state)
print(ra.get_position(ra.cylinder_handle))


ra.goto_position(state)
ra.enable_claw(True)
ra.goto_position([-0.3, -0.11, 0.12])


"""

episodes = 1000

while episodes > 0:
    print('=============================================> Episode ', episodes)
    raa.init()
    i = 0
    while i < 100:
        terminate1 = raa.step_through()
        if terminate1:
            break
        i += 1

    raa.save_qtable()
    episodes -= 1
