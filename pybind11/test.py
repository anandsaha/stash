#!/usr/bin/python3.5

import example
import numpy as np

print("Method invocation ------------")
print(example.add(88, 76))

print("Attributes -------------------")
example.the_answer = 41
print(example.the_answer)
print(example.what)

print("Class ------------------------")

p = example.Pet('Cat')
print(p)
print(p.getName())
p.setName('Dog')
print(p.getName())

p.name = 'Tiger'
print(p.name)

print('Numpy Array ------------------')
a = np.arange(12).reshape(3, 2, 2).astype('float')
print(a)
print(example.sum(a))
example.twice(a)
print(a)
