# I am a comment, and I want to say that the variable CC will be
# the compiler to use.
CC=g++
# Hey!, I am comment number 2. I want to say that CFLAGS will be the
# options I'll pass to the compiler.
CFLAGS=-g -c -std=c++0x

INCLUDEFLAGS=-I/home/hoe01h/Downloads/trajopt/src -I/usr/include/eigen3 -I/usr/local/include/openrave-0.9 -I/usr/include/boost -L/usr/lib/ -L/usr/local/lib/

LDFLAGS= -lutils -ltrajopt -lopenrave0.9 -lopenrave0.9-core -losgviewer -lsco
BOOSTFLAGS= -lboost_system -lboost_program_options

all: traj clean

traj: traj.o
	$(CC) $(INCLUDEFLAGS) traj.o -o traj $(LDFLAGS) $(BOOSTFLAGS)

traj.o: traj.cpp
	$(CC) $(CFLAGS) $(INCLUDEFLAGS) traj.cpp
	

clean:
	rm *.o
