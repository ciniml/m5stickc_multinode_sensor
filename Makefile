.PHONY: all clean

CXXFLAGS := -O0
LDFLAGS :=

%.cpp: %.o
	g++ $(CXXFLAGS) -c -o %@ %<

master: main_unix_master.o
	g++ $(LDFLAGS) $^ -o $@

clean:
	-@$(RM) *.o
	-@$(RM) master
