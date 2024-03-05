# Compiler settings - Can change to clang++ if preferred
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -g -I.

# Linker flags
LDFLAGS =

# Source files
SOURCES = ik.cpp linalg.cpp main.cpp methods.cpp new_methods.cpp structs.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
EXEC = myExecutable

# Default target
all: $(EXEC)

$(EXEC): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^

# To obtain object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# To remove generated files
clean:
	rm -f $(EXEC) $(OBJECTS)
