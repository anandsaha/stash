from robot import RobotArm
import numpy as np
import copy
import os
import utility


class RobotArmAgent(object):
    def __init__(self, robot, alpha=0.1, gamma=0.9, epsilon=0.2, q_init=-1):

        self.robot = robot
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon

        self.tolerance = utility.rnd(0.01)

        # The actions the agent can take - either goto some x,y,z position 
        # or engage/disengage claw
        dim = self.robot.get_env_dimensions()
        x_range = np.arange(dim[0][0], dim[0][1], self.tolerance)
        y_range = np.arange(dim[1][0], dim[1][1], self.tolerance)
        z_range = np.arange(dim[2][0], dim[2][1], self.tolerance)

        self.action_type1 = 'move_gripper'
        self.action_type2 = 'engage_gripper'
        self.invalid_state = [-100, -100, -100, False]

        # Actions consist of
        # a) Gripper Enable/Disable
        # b) Goto location (x, y, z)
        self.actions = []
        self.actions.append([self.action_type2, True])
        self.actions.append([self.action_type2, False])

        for x in x_range[1:-1]:
            for y in y_range[1:-1]:
                for z in z_range[1:-1]:
                    self.actions.append([self.action_type1, [x, y, z]])

        self.total_actions = len(self.actions)

        # The states the environment can be in, is made up of all the locations and if gripper is holding
        # object or not
        self.states = []
        for x in x_range:
            for y in y_range:
                for z in z_range:
                    for b in [True, False]:
                        self.states.append([x, y, z, b])

        self.states.append(self.invalid_state)  # invalid state, the last state.
        self.total_states = len(self.states)
        self.invalid_states_index = self.total_states - 1

        print("There are {0} actions.".format(self.total_actions))
        print("There are {0} states.".format(self.total_states))

        self.q_table = np.full((self.total_states, self.total_actions), q_init)

        # Give the claw engaging actions slight higher initial values
        for index, val in enumerate(self.q_table):
            if not self.states[index][3]:
                self.q_table[index][0] += 1
            else:
                self.q_table[index][1] += 1

        self.state_id = None
        self.actionstate_prev = {}
        self.actionstate_curr = {}

    def load_qtable(self):
        f = 'qtables/qtable.txt.npy'
        if os.path.exists(f):
            self.q_table = np.load(f)

    def save_qtable(self):
        f = 'qtables/qtable.txt.npy'
        if os.path.exists(f):
            os.remove(f)
        np.save(f, self.q_table)

    def init(self):
        self.robot.restart_sim()
        # Place the arm in a well defined position within the state space
        self.robot.goto_position([utility.rnd(-0.30), utility.rnd(-0.10), utility.rnd(0.14)])
        self.state_id = self.observe_state()
        self.actionstate_prev = {}
        self.actionstate_curr = {}

    def get_canonical_position(self, handle, gripper_holding_object):
        pos = self.robot.get_position(handle)

        state_id = 0
        for state in self.states:
            if abs(state[0] - pos[0]) < self.tolerance \
                    and abs(state[1] - pos[1]) < self.tolerance \
                    and abs(state[2] - pos[2]) < self.tolerance:
                if state[3] == gripper_holding_object:
                    return state_id
            state_id += 1

        print('Position was invalid ', pos)
        return self.invalid_states_index  # The last state, which is the invalid state, will get returned

    def update_actionstate(self, action_id):
        self.actionstate_curr['action_id'] = action_id
        self.actionstate_curr['state_id'] = self.observe_state()
        self.actionstate_curr['state'] = self.states[self.actionstate_curr['state_id']]
        self.actionstate_curr['cylinder_position'] = self.robot.get_position(self.robot.cylinder_handle)
        self.actionstate_curr['bin_position'] = self.robot.get_position(self.robot.bin_handle)
        self.actionstate_curr['claw_position'] = self.robot.get_position(self.robot.gripper_handle)
        self.actionstate_curr['claw_cylinder_distance'] = utility.distance(self.actionstate_curr['claw_position'],
                                                                           self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['bin_cylinder_distance'] = utility.distance(self.actionstate_curr['bin_position'],
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
            print('Action: Moving claw', action[1])
            self.robot.goto_position(action[1])
        elif action[0] == self.action_type2:
            print('Action: Engaging/Disengaging claw', action[1])
            self.robot.enable_grip(action[1])

    def update_q_table(self, state, action, reward, state_new):
        print("Reward: ", reward)
        q_sa = self.q_table[state, action]
        td_error = reward + self.gamma * np.max(self.q_table[state_new]) - q_sa
        self.q_table[state, action] = q_sa + self.alpha * td_error
        #print ("Q-Value:", state, action, self.q_table[state, action])

    def observe_state(self):
        return self.get_canonical_position(self.robot.gripper_handle, self.robot.is_object_held())

    def calculate_reward(self):

        state_new = self.actionstate_curr['state']

        terminate = False
        is_pass = False

        if state_new == self.invalid_state:
            reward = -10
            print('Penalty: Reached invalid state, terminating')
            terminate = True
        elif (ra.get_position(ra.cylinder_handle)[2] - ra.cylinder_z_locus) < (-1 * self.tolerance):
            reward = -10
            print('Penalty: Cylinder has fallen, terminating')
            terminate = True
        elif utility.distance(ra.bin_position, self.actionstate_curr['bin_position']) > self.tolerance:
            reward = -10
            print('Penalty: Bin has shifted, terminating')
            terminate = True
        elif len(self.actionstate_prev) > 0 and self.actionstate_curr['state'] == self.actionstate_prev['state']:
            reward = -1
            print('Penalty: Previous and current state is same')
        elif self.robot.gripper_enabled \
                and not self.actionstate_curr['is_cylinder_held'] \
                and not self.actionstate_curr['cylinder_in_bin']:
            reward = -1
            print('Penalty: Claw is engaged but cylinder is not in claw')
        elif len(self.actionstate_prev) > 0 \
                and self.actionstate_prev['is_cylinder_held'] \
                and not self.actionstate_curr['is_cylinder_held'] \
                and not self.actionstate_curr['cylinder_in_bin']:
            reward = -1
            print('Penalty: Claw did not drop the cylinder in the bin')
        elif self.robot.gripper_enabled \
                and self.actionstate_curr['is_cylinder_held'] \
                and not self.actionstate_curr['cylinder_in_bin']:
            if (ra.get_position(ra.cylinder_handle)[2] - ra.cylinder_z_locus) > self.tolerance:
                reward = 10
                print('Reward: Claw could grab *and lift* the cylinder !!!!!!!!!!!!!!!!!!!!!!!!!')
            else:
                reward = 5
                print('Reward: Claw could grab the cylinder !!!!!!!!!!!!!!!!!!!!!!!!!')
        elif self.actionstate_curr['cylinder_in_bin']:
            reward = 100
            print('Reward: Cylinder in bucket !!!!!!!!!!!!!!!!!!!!!!!!!')
            terminate = True
            is_pass = True
        else:
            reward = -1

        return reward, terminate, is_pass

    def step_through(self):
        self.actionstate_prev = copy.deepcopy(self.actionstate_curr)
        action_id = self.choose_action(self.state_id)
        self.do_action(action_id)
        self.update_actionstate(action_id)
        state_id_new = self.actionstate_curr['state_id']

        reward, terminate, is_pass = self.calculate_reward()

        # I was in this state, I took this action, I got this reward, and I reached the new state
        self.update_q_table(self.state_id, action_id, reward, state_id_new)
        self.state_id = state_id_new
        return terminate, is_pass, reward


ra = RobotArm('127.0.0.1', 19997)
raa = RobotArmAgent(ra, epsilon=0.2, q_init=-1)
episodes = 20000
episode_num = 0
raa.load_qtable()

while episodes > 0:
    print('=============================================> Episode ', episodes)
    raa.init()
    i = 0
    is_pass = False
    total_reward = 0
    episode_num += 1
    while i < 100:
        terminate_x, is_pass_x, reward = raa.step_through()
        total_reward += reward
        if terminate_x:
            is_pass = is_pass_x
            break
        i += 1

    # save episode_id, is_pass, total_reward, (i+1)
    stat_file = open('qtables/1aug.txt', 'a')
    stat_file.write("{0}, {1}, {2}, {3}\n".format(episode_num, is_pass, total_reward, i+1))
    stat_file.close()
    raa.save_qtable()
    episodes -= 1
