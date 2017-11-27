CXX=g++
CXXFLAGS=-Wall -O2
LDFLAGS=
INCLUDE=-I.

CXXSRCS = $(wildcard *.cpp)

CXXOBJS = $(CXXSRCS:.cpp=.o)
OBJECTS = $(CXXOBJS)
TARGET = assign

all: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $(OBJECTS) -o $(TARGET)

clean:
	rm -rf $(OBJECTS) $(TARGET)
