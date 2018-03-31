CPPFLAGS=-std=c++11 -O3 -DNDEBUG
LDFLAGS=-lSDL2 -lSDL2_gfx
OBJS=drawing.o simple_network_builder.o graphml_network_builder.o random.o

STANDARD=traffic.o graphml_simulation.o
AOS_INT=traffic_aos_int.o traffic.o graphml_simulation_aos_int.o
AOS_INT_CUDA=traffic_aos_int_cuda.o traffic_aos_int.o traffic.o graphml_simulation_aos_int_cuda.o

CXX=/usr/local/cuda/bin/nvcc

graphml_simulation_aos_int_cuda.o: graphml_simulation_aos_int_cuda.cu
	$(CXX) $(CPPFLAGS) -c $<

traffic_aos_int_cuda.o: traffic_aos_int_cuda.cu traffic_aos_int_cuda.h
	$(CXX) $(CPPFLAGS) -c $<

random.o: random.cu random.h
	$(CXX) $(CPPFLAGS) -c $<

%.o: %.cc %.h
	$(CXX) $(CPPFLAGS) -c $<

graphml: $(OBJS) $(STANDARD) traffic_logic.inc
	$(CXX) -I. -Ilib/rapidxml-1.13 -o simulation $(OBJS) $(STANDARD) $(LDLIBS) $(LDFLAGS)

graphml_aos_int: $(OBJS) $(AOS_INT) graphml_simulation.o traffic_logic.inc
	$(CXX) -I. -Ilib/rapidxml-1.13 -o graphml_simulation_aos_int $(OBJS) $(AOS_INT) $(LDLIBS) $(LDFLAGS)

graphml_aos_int_cuda: $(OBJS) $(AOS_INT_CUDA) graphml_simulation_aos_int_cuda.o traffic_logic.inc
	$(CXX) -I. -Ilib/rapidxml-1.13 -o graphml_simulation_aos_int_cuda $(OBJS) $(AOS_INT_CUDA) $(LDLIBS) $(LDFLAGS)

clean:
	rm -f *.o simulation graphml_simulation graphml_simulation_aos_int
