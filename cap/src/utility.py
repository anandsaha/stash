import numpy as np
import decimal


def distance(pos1, pos2):
    x2 = np.square(pos1[0] - pos2[0])
    y2 = np.square(pos1[1] - pos2[1])
    z2 = np.square(pos1[2] - pos2[2])

    return np.sqrt(x2 + y2 + z2)


def rnd(val, to='0.01'):
    return decimal.Decimal(val).quantize(decimal.Decimal(to), decimal.ROUND_HALF_DOWN)
