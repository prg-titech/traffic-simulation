CPPFLAGS=-std=c++11
LDFLAGS=-lgraph
OBJS=drawing.o simple_network_builder.o simulation.o traffic.o

%.o: %.cc
	$(CXX) $(CPPFLAGS) -c $<

simulation: $(OBJS)
	$(CXX) -o simulation $(OBJS) $(LDLIBS) $(LDFLAGS)

clean:
	rm -f *.o simulation
