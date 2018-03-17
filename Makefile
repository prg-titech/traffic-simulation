CPPFLAGS=-std=c++11
LDFLAGS=-lSDL2 -lSDL2_gfx
OBJS=drawing.o simple_network_builder.o graphml_network_builder.o traffic.o random.o

%.o: %.cc %.h
	$(CXX) $(CPPFLAGS) -c $<

graphml: $(OBJS) graphml_simulation.o
	$(CXX) -I. -Ilib/rapidxml-1.13 -o simulation $(OBJS) graphml_simulation.o $(LDLIBS) $(LDFLAGS)

clean:
	rm -f *.o simulation graphml_simulation
