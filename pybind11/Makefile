CC = g++
CFLAGS = -O3 -shared -std=c++11 -I ~/store/git/pybind/pybind11/include/ -I /usr/include/python3.5  `python3-config --cflags --ldflags` -fPIC 

example.so:
	$(CC) $(CFLAGS) example.cpp -o example.so

c:
	rm *.so
