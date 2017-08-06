import numpy as np
import copy
import os
import utility
from utility import log_and_display
import reward_strategy


class RobotArmAgent(object):
    def __init__(self, robot, learn_rate=0.1, discount=0.9, epsilon=0.2, q_init_val=0.0):

        self.robot = robot
        self.learn_rate = learn_rate
        self.discount = discount
        self.epsilon = epsilon

        self.tolerance = utility.rnd(0.01)

        dim = self.robot.get_env_dimensions()
        x_range = np.arange(dim[0][0], dim[0][1], self.tolerance*2)
        y_range = np.arange(dim[1][0], dim[1][1], self.tolerance*2)
        z_range = np.arange(dim[2][0], dim[2][1], self.tolerance*2)

        print(x_range)
        print(y_range)
        print(z_range)

        # The actions the agent can take - either goto some x,y,z position
        # or engage/disengage claw
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

        x_range = np.arange(dim[0][0], dim[0][1], self.tolerance)
        y_range = np.arange(dim[1][0], dim[1][1], self.tolerance)
        z_range = np.arange(dim[2][0], dim[2][1], self.tolerance)

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

        log_and_display("There are {0} actions.".format(self.total_actions))
        log_and_display("There are {0} states.".format(self.total_states))

        self.q_table = np.full((self.total_states, self.total_actions), q_init_val)

        # Give the claw engaging actions slight higher initial values
        for index, val in enumerate(self.q_table):
            if not self.states[index][3]:  # If grip not enabled
                self.q_table[index][0] += 1  # Encourage grip enable
                self.q_table[index][1] -= 10
            else:
                self.q_table[index][1] += 1
                self.q_table[index][0] -= 10

        self.current_state_id = None
        self.actionstate_prev = {}
        self.actionstate_curr = {}
        self.episode_object_gripped = False

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
        log_and_display('Initializing episode')
        self.robot.restart_sim()
        # Place the arm in a well defined position within the state space
        self.robot.goto_position([utility.rnd(-0.30), utility.rnd(-0.10), utility.rnd(0.14)])
        self.current_state_id = self.observe_state()
        self.actionstate_prev = {}
        self.actionstate_curr = {}
        self.episode_object_gripped = False

    def get_canonical_position(self, handle, gripper_holding_object):
        pos = self.robot.get_position(handle)

        current_state_id = 0
        for state in self.states:
            if abs(state[0] - pos[0]) < self.tolerance \
                    and abs(state[1] - pos[1]) < self.tolerance \
                    and abs(state[2] - pos[2]) < self.tolerance \
                    and state[3] == gripper_holding_object:
                        return current_state_id
            current_state_id += 1

        log_and_display('--------------------> Position was invalid ' + str(pos))
        return self.invalid_states_index  # The last state, which is the invalid state, will get returned

    def update_actionstate(self, action_id):
        self.actionstate_curr['action_id'] = action_id
        self.actionstate_curr['current_state_id'] = self.observe_state()
        self.actionstate_curr['state'] = self.states[self.actionstate_curr['current_state_id']]
        self.actionstate_curr['cylinder_position'] = self.robot.get_position(self.robot.cylinder_handle)
        self.actionstate_curr['bin_position'] = self.robot.get_position(self.robot.bin_handle)
        self.actionstate_curr['claw_position'] = self.robot.get_position(self.robot.gripper_handle)
        self.actionstate_curr['claw_cylinder_distance'] = utility.distance(self.actionstate_curr['claw_position'],
                                                                           self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['bin_cylinder_distance'] = utility.distance(self.actionstate_curr['bin_position'],
                                                                          self.actionstate_curr['cylinder_position'])
        self.actionstate_curr['cylinder_in_bin'] = self.robot.is_object_in_bin()
        self.actionstate_curr['is_cylinder_held'] = self.robot.is_object_held()

    def choose_action(self, current_state_id):
        if np.random.uniform() < self.epsilon:
            action_id = np.random.choice(self.total_actions)
        else:
            action_id = np.argmax(self.q_table[current_state_id])
        return action_id

    def do_action(self, action_id):
        action = self.actions[action_id]

        if action[0] == self.action_type1:
            log_and_display('Action: Moving claw ' + str(action[1]))
            self.robot.goto_position(action[1])
        elif action[0] == self.action_type2:
            log_and_display('Action: Engaging/Disengaging claw ' + str(action[1]))
            self.robot.enable_grip(action[1])

    def update_q_table(self, state, action, reward, state_new):
        q_sa = self.q_table[state, action]
        td_error = reward + self.discount * np.max(self.q_table[state_new]) - q_sa
        self.q_table[state, action] = q_sa + self.learn_rate * td_error
        msg = "Q-Value: S:{}, A:{}, R:{}, S`:{}, TE: {}, Q:{}, Q`:{}".format(state, action, reward, state_new, td_error,
                                                                             q_sa, self.q_table[state, action])
        log_and_display(msg)

    def observe_state(self):
        return self.get_canonical_position(self.robot.gripper_handle, self.robot.is_object_held())

    def step_through(self):
        self.actionstate_prev = copy.deepcopy(self.actionstate_curr)
        action_id = self.choose_action(self.current_state_id)
        self.do_action(action_id)
        self.update_actionstate(action_id)
        new_state_id = self.actionstate_curr['current_state_id']

        reward, terminate, is_pass = reward_strategy.calculate_reward(self)

        # I was in this state, I took this action, I got this reward, and I reached the new state
        self.update_q_table(self.current_state_id, action_id, reward, new_state_id)
        self.current_state_id = new_state_id
        return terminate, is_pass, reward

