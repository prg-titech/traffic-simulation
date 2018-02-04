CPPFLAGS=-std=c++11
LDFLAGS=-lSDL2 -lSDL2_gfx
OBJS=drawing.o simple_network_builder.o traffic.o

%.o: %.cc
	$(CXX) $(CPPFLAGS) -c $<

simulation: $(OBJS) simulation.o
	$(CXX) -o simulation $(OBJS) simulation.o $(LDLIBS) $(LDFLAGS)

graphml: $(OBJS) graphml_simulation.o
	$(CXX) -I. -Ilib/rapidxml-1.13 -o simulation $(OBJS) graphml_simulation.o $(LDLIBS) $(LDFLAGS)

clean:
	rm -f *.o simulation graphml_simulation
