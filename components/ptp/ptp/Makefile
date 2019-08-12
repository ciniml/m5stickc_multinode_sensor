.PHONY: all clean

BOOSTDIR ?= $(HOME)/boost_1_70_0

CXXFLAGS := -O0 -I$(BOOSTDIR) -g -std=c++11
LDFLAGS :=

all: master slave

%.o: %.cpp
	g++ $(CXXFLAGS) -c -o $@ $<

master: main_unix_master.o
	g++ $(LDFLAGS) $^ -o $@

slave: main_unix_slave.o
	g++ $(LDFLAGS) $^ -o $@

clean:
	-@$(RM) *.o
	-@$(RM) master
