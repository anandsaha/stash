from agent import Agent
from utility import log_and_display
from environment import Environment
import numpy as np
import config


vrep_ip = '127.0.0.1'
vrep_port = 19997

env = Environment(vrep_ip, vrep_port)

agent = Agent(env, epsilon=config.EPSILON, q_init_val=config.Q_INIT_VAL,
              discount=config.DISCOUNT, learn_rate=config.LEARN_RATE)

episodes = config.NUM_EPISODES
agent.load_qtable()

log_and_display("%%%%%%%%%%%%%%%%%%%%%%%%% Main starts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
log_and_display("Epsilon: " + str(config.EPSILON))
log_and_display("Epsilon Decay: " + str(config.EPSILON_DECAY))
log_and_display("Q Init: " + str(config.Q_INIT_VAL))
log_and_display("Discount: " + str(config.DISCOUNT))
log_and_display("Learning Rate: " + str(config.LEARN_RATE))
log_and_display("Max Episodes: " + str(config.NUM_EPISODES))
log_and_display("Max Actions/Episodes: " + str(config.NUM_MAX_ACTIONS))
log_and_display("Env Dimensions: " + str(config.ENV_DIMENSION))

while episodes > 0:
    log_and_display('=============================================> Episode ' + str(episodes))
    agent.reset()
    total_reward, total_steps, success, total_explorations = agent.execute_episode_qlearn(config.NUM_MAX_ACTIONS)
    agent.epsilon -= np.maximum(0.0, config.EPSILON_DECAY)  # Reduce exploration rate with each episode
    episode_num = config.NUM_EPISODES - episodes + 1

    stat_file = open(config.PLOT_FILE, 'a')
    stat_file.write("{0}, {1}, {2}, {3}, {4}\n".format(episode_num, success, total_reward,
                                                       total_steps, total_explorations))
    stat_file.close()
    agent.save_qtable()
    episodes -= 1

    if success and total_steps == config.MIN_ACTIONS_EXPECTED and total_explorations == 0:
        log_and_display('Optimal moves learnt. Terminating training. Now run agent with epsilon as 0.')
        break

