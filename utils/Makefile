CFLAGS = -ggdb -O2

all: vec_file.o test_vec_file
	touch .tstamp

vec_file.o: vec_file.cpp vec_file.h
test_vec_file: vec_file.cpp vec_file.h
	g++ -DTEST_VEC_FILE=1 -o test_vec_file vec_file.cpp

clean: 
	- \rm *.o 
	- \rm test_vec_file
