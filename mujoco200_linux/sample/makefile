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
closed_chain_control:
	gcc -c $(COMMON) -I../../../eigen-master -I../../../CBLAS/include -I../../../CBLAS/src -I../../../lapack-3.9.1/LAPACKE/src -I../../../lapack-3.9.1/LAPACKE/include -I../../../lowlevelapi_test closed_chain_control.c -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 ../../../lowlevelapi_test/libtest.o ../../../lowlevelapi_test/matram.o
	gfortran -I../include -L../bin -o ../bin/closed_chain_control closed_chain_control.o ../../../lowlevelapi_test/libtest.o ../../../lowlevelapi_test/matram.o ../../../lowlevelapi_test/pbplots/pbPlots.o ../../../lowlevelapi_test/pbplots/supportLib.o ../../../CBLAS/lib/cblas_LINUX.a ../../../CBLAS/lib/libblas.a  ../../../CBLAS/lib/liblapacke.a ../../../CBLAS/lib/libcblas.a ../bin/libmujoco200.so ../bin/libglew.so -llapack -lmujoco200 -lGL -lglew ../bin/libglfw.so.3
digit_walking_control:
	gcc -c $(COMMON) -I../../../CBLAS/include -I../../../CBLAS/src -I../../../lapack-3.9.1/LAPACKE/src -I../../../lapack-3.9.1/LAPACKE/include -I../../../lowlevelapi_test digit_walking_control_test.c -lmujoco200 -lGL -lglew ../bin/libglfw.so.3 ../../../lowlevelapi_test/libtest.o ../../../lowlevelapi_test/matram.o
	gfortran -I../include -L../bin -o ../bin/digit_walking_control_test digit_walking_control_test.o ../../../lowlevelapi_test/libtest.o ../../../lowlevelapi_test/matram.o ../../../lowlevelapi_test/pbplots/pbPlots.o ../../../lowlevelapi_test/pbplots/supportLib.o ../../../CBLAS/lib/cblas_LINUX.a ../../../CBLAS/lib/libblas.a  ../../../CBLAS/lib/liblapacke.a ../../../CBLAS/lib/libcblas.a ../bin/libmujoco200.so ../bin/libglew.so -llapack -lmujoco200 -lGL -lglew ../bin/libglfw.so.3
