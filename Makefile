# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-c -I/home/marcus/trajopt/src -I/home/marcus/trajopt/src/trajopt -I/usr/include/eigen3 -I/usr/local/include/openrave-0.9 -I/usr/include/boost -L/home/marcus/trajopt/build_trajopt/lib -L/usr/lib/x86_64-linux-gnu

LDFLAGS= -lutils -ltrajopt
BOOSTFLAGS= -lboost_system 

all: traj clean

traj.o: traj.cpp
	$(CC) $(CFLAGS) traj.cpp $(LDFLAGS)
	
traj: traj.o
	$(CC) traj.o -o traj $(BOOSTFLAGS)

clean:
	rm *o traj
