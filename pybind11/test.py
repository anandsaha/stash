#!/usr/bin/python3.5

import minimalml as ml
import example as e
import numpy as np

print('Numpy Array ------------------')
a = np.arange(12).reshape(3, 4).astype('float')
print(e.sum(a))
print(a)
ml.show(a)
