from utility import log_and_display
import utility


def is_valid_state(agent):
    if agent.actionstate_curr['state'] == agent.invalid_state:
        return False
    return True


def is_cylinder_standing(agent):
    cylinder_pos = agent.robot.get_position(agent.robot.cylinder_handle)
    if (cylinder_pos[2] - agent.robot.cylinder_z_locus) <= (-1 * agent.tolerance):
        return False
    return True


def is_bin_inplace(agent):
    if utility.distance(agent.robot.bin_position, agent.actionstate_curr['bin_position']) > agent.tolerance:
        return False
    return True


def is_previous_current_state_same(agent):
    if len(agent.actionstate_prev) > 0 and agent.actionstate_curr['state'] == agent.actionstate_prev['state']:
        return True
    return False


def is_grip_engaged_with_no_object(agent):
    if agent.robot.gripper_enabled \
            and not agent.actionstate_curr['is_cylinder_held'] \
            and not agent.actionstate_curr['cylinder_in_bin']:
        return True
    return False


def is_cylinder_not_dropped_in_bin(agent):
    if len(agent.actionstate_prev) > 0 \
            and agent.actionstate_prev['is_cylinder_held'] \
            and not agent.actionstate_curr['is_cylinder_held'] \
            and not agent.actionstate_curr['cylinder_in_bin']:
        return True
    return False


def is_grip_holding_object(agent):
    if not agent.episode_object_gripped \
            and agent.robot.gripper_enabled \
            and agent.actionstate_curr['is_cylinder_held'] \
            and not agent.actionstate_curr['cylinder_in_bin']:
        agent.episode_object_gripped = True
        return True
    return False


def is_object_in_bin(agent):
    if len(agent.actionstate_prev) > 0 \
            and agent.actionstate_prev['is_cylinder_held'] \
            and agent.actionstate_curr['cylinder_in_bin'] \
            and is_cylinder_standing(agent)\
            and not is_grip_holding_object(agent) \
            and is_bin_inplace(agent):
        return True
    return False


def calculate_reward(agent):
    """return reward, terminate_episode, is_pass"""

    if not is_valid_state(agent):
        log_and_display('Penalty: Reached invalid state, terminating')
        return -100, True, False

    if not is_cylinder_standing(agent):
        log_and_display('Penalty: Cylinder has fallen, terminating')
        return -100, True, False

    if not is_bin_inplace(agent):
        log_and_display('Penalty: Bin has shifted, terminating')
        return -100, True, False
    # Disabled
    if False and is_previous_current_state_same(agent):
        log_and_display('Penalty: Previous and current state is same')
        return -100, False, False

    if is_grip_engaged_with_no_object(agent):
        log_and_display('Penalty: Claw is engaged but cylinder is not in claw')
        return -100, False, False

    if is_cylinder_not_dropped_in_bin(agent):
        log_and_display('Penalty: Claw did not drop the cylinder in the bin')
        return -100, False, False

    if is_grip_holding_object(agent):
        log_and_display('Reward: Claw could grab the cylinder for first time')
        return 5, False, False

    if is_object_in_bin(agent):
        log_and_display('Reward: Cylinder in bucket. Objective achieved !!!!!!!!')
        return 100, True, True

    return -2, False, False  # Default
