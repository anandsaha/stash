from agent import Agent
from utility import log_and_display
from environment import Environment
import config


vrep_ip = '127.0.0.1'
vrep_port = 19997
agent = Agent(Environment(vrep_ip, vrep_port), learn_rate=0.5, discount=0.9, epsilon=0.2, q_init_val=0.0)
episodes = 1000
episode_num = 0
agent.load_qtable()

log_and_display("%%%%%%%%%%%%%%%%%%%%%%%%% Main starts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

while episodes > 0:
    log_and_display('=============================================> Episode ' + str(episodes))
    total_reward, total_steps, success = agent.execute_episode(100)
    agent.epsilon -= 0.00005  # Reduce exploration rate with each episode
    episode_num += 1

    stat_file = open(config.PLOT_FILE, 'a')
    stat_file.write("{0}, {1}, {2}, {3}\n".format(episode_num, success, total_reward, total_steps))
    stat_file.close()
    agent.save_qtable()
    episodes -= 1
    agent.reset()
