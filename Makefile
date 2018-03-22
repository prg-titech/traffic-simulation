CPPFLAGS=-std=c++11
LDFLAGS=-lSDL2 -lSDL2_gfx
OBJS=drawing.o simple_network_builder.o graphml_network_builder.o random.o

STANDARD=traffic.o graphml_simulation.o
AOS_INT=traffic_aos_int graphml_simulation_aos_int.o

%.o: %.cc %.h
	$(CXX) $(CPPFLAGS) -c $<

graphml: $(OBJS) $(STANDARD) traffic_logic.inc
	$(CXX) -I. -Ilib/rapidxml-1.13 -o simulation $(OBJS) $(STANDARD) $(LDLIBS) $(LDFLAGS)

graphml_aos_int: $(OBJS) $(AOS_INT) graphml_simulation.o traffic_logic.inc
	$(CXX) -I. -Ilib/rapidxml-1.13 -o graphml_simulation_aos_int $(OBJS) $(AOS_INT) $(LDLIBS) $(LDFLAGS)

clean:
	rm -f *.o simulation graphml_simulation graphml_simulation_aos_int
