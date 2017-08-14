from agent import Agent
from utility import log_and_display
from environment import Environment
import config


vrep_ip = '127.0.0.1'
vrep_port = 19997

env = Environment(vrep_ip, vrep_port)

agent = Agent(env, epsilon=config.EPSILON, q_init_val=config.Q_INIT_VAL,
              discount=config.DISCOUNT, learn_rate=config.LEARN_RATE)

episodes = config.NUM_EPISODES
agent.load_qtable()

log_and_display("%%%%%%%%%%%%%%%%%%%%%%%%% Main starts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

while episodes > 0:
    log_and_display('=============================================> Episode ' + str(episodes))
    total_reward, total_steps, success, total_explorations = agent.execute_episode_qlearn(config.NUM_MAX_ACTIONS)
    agent.epsilon -= config.EPSILON_DECAY  # Reduce exploration rate with each episode
    episode_num = config.NUM_EPISODES - episodes + 1

    stat_file = open(config.PLOT_FILE, 'a')
    stat_file.write("{0}, {1}, {2}, {3}\n".format(episode_num, success, total_reward, total_steps))
    stat_file.close()
    agent.save_qtable()
    episodes -= 1
    agent.reset()

    if success and total_steps == 5 and total_explorations == 0:
        log_and_display('Optimal moves learnt. Terminating training. Now run agent with epsilon as 0.')
        break

