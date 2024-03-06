# Compiler settings - Can change to clang++ if preferred
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -g -I.

# Linker flags
LDFLAGS =

# Source files
SOURCES = ik.cpp linalg.cpp main.cpp methods.cpp new_methods.cpp structs.cpp

# Executable name
EXEC = myExecutable

# Default target
all:
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) -o $(EXEC)

# To remove generated executable
clean:
	rm -f $(EXEC) *.o
