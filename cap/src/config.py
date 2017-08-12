import utility

Q_TABLE_FILE = 'qtables/qtable.txt.npy'
PLOT_FILE = 'qtables/plot.txt'

# Adjustable parameters
INVALID_STATE = [-100, -100, -100, False]
UNIT_STEP_SIZE = 0.02
TOLERANCE = 0.01
TOLERANCE2 = 0.005

INIT_ARM_POSITION = [utility.rnd(-0.30), utility.rnd(-0.10), utility.rnd(0.14)]
ENV_DIMENSION = [[utility.rnd(-0.32), utility.rnd(-0.20)],
                 [utility.rnd(-0.12), utility.rnd(-0.07)],
                 [utility.rnd(0.0), utility.rnd(0.11)]]
