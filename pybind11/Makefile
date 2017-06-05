CC = g++
CFLAGS = -O3 -shared -std=c++11 -I ~/store/git/pybind/pybind11/include/ -I /usr/include/python3.5  `python3-config --cflags --ldflags` -fPIC 

example.so:
	$(CC) $(CFLAGS) example.cpp -o example.so

minimalml.so:
	$(CC) $(CFLAGS) minimalml.cpp -o minimalml.so

all: minimalml.so example.so

c:
	rm *.so
