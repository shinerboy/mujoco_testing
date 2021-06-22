COMMON=-O2 -I../include -L../bin -std=c++11 -mavx -pthread -Wl,-rpath,'$$ORIGIN'

all:
	g++ $(COMMON) testxml.cpp    -lmujoco200nogl                             -o ../bin/testxml
	g++ $(COMMON) testspeed.cpp  -lmujoco200nogl                             -o ../bin/testspeed
	g++ $(COMMON) compile.cpp    -lmujoco200nogl                             -o ../bin/compile
	g++ $(COMMON) derivative.cpp -lmujoco200nogl -fopenmp                    -o ../bin/derivative
	g++ $(COMMON) basic.cpp      -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/basic
	g++ $(COMMON) record.cpp     -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/record
	gcc -c -O2 -mavx -I../include ../include/uitools.c
	g++ $(COMMON) uitools.o simulate.cpp -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/simulate
	g++ $(COMMON) uitools.o simulate_test.cpp -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/simulate_test
	g++ $(COMMON) pd.cpp      -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/pd
	rm *.o

digit_leg: 
	gcc $(COMMON) digit_leg.c      -lmujoco200 -lGL -lglew -lm ../bin/libglfw.so.3 -o ../bin/digit_leg
closed_chain:
	gcc $(COMMON) closed_chain.c      -lmujoco200 -lGL -lglew -lm ../bin/libglfw.so.3 -o ../bin/closed_chain
