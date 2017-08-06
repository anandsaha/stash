from agent import RobotArmAgent
from robot import RobotArm
from utility import log_and_display

vrep_ip = '127.0.0.1'
vrep_port = 19997

ra = RobotArm(vrep_ip, vrep_port)
raa = RobotArmAgent(ra, learn_rate=0.5, discount=0.9, epsilon=0., q_init_val=0.0)
episodes = 100
episode_num = 0
raa.load_qtable()
log_and_display("%%%%%%%%%%%%%%%%%%%%%%%%% Run starts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
while episodes > 0:
    log_and_display('=============================================> Episode ' + str(episodes))
    raa.init()
    i = 0
    is_pass = False
    episode_reward = 0
    episode_num += 1
    while i < 100:
        terminate_x, is_pass_x, reward = raa.step_through()
        episode_reward += reward
        if terminate_x:
            is_pass = is_pass_x
            break
        i += 1

    raa.epsilon -= 0.00005  # Reduce exploration rate with each episode

    # save episode_id, is_pass, episode_reward, (i+1)
    stat_file = open('qtables/1aug.txt', 'a')
    stat_file.write("{0}, {1}, {2}, {3}\n".format(episode_num, is_pass, episode_reward, i + 1))
    stat_file.close()
    raa.save_qtable()
    episodes -= 1
