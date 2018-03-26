# Copyright (C) 2003, 2010 International Business Machines and others.
# All Rights Reserved.
# This file is distributed under the Eclipse Public License.

# $Id: Makefile.in 2016 2011-06-15 08:53:32Z stefan $

##########################################################################
#    You can modify this example makefile to fit for your own program.   #
#    Usually, you only need to change the five CHANGEME entries below.   #
##########################################################################

# CHANGEME: This should be the name of your executable
EXE = get_started 

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement
OBJS =  get_started.o ipopt_solve.o ode_inverse.o retape.o

# CHANGEME: Additional libraries
ADDLIBS =

# CHANGEME: Additional flags for compilation (e.g., include flags)
ADDINCFLAGS = -I/home/jom/Documents/prgs/projects/cppad -I/home/jom/Documents/prgs/projects/Ipopt-3.12.9/include

##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++

# C++ Compiler options
CXXFLAGS = -O3 -pipe -DNDEBUG -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -Wno-unknown-pragmas -Wno-long-long   -DIPOPT_BUILD

# additional C++ Compiler options for linking
CXXLINKFLAGS =  -Wl,--rpath -Wl,/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib

# Include directories (we use the CYGPATH_W variables to allow compilation with Windows compilers)
INCL = `PKG_CONFIG_PATH=/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib64/pkgconfig:/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib/pkgconfig:/home/jom/Documents/prgs/projects/Ipopt-3.12.9/share/pkgconfig:/opt/ros/kinetic/lib/pkgconfig pkg-config --cflags ipopt` $(ADDINCFLAGS)
#INCL = -I`$(CYGPATH_W) /home/jom/Documents/prgs/projects/Ipopt-3.12.9/include/coin`  $(ADDINCFLAGS)

# Linker flags
LIBS = `PKG_CONFIG_PATH=/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib64/pkgconfig:/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib/pkgconfig:/home/jom/Documents/prgs/projects/Ipopt-3.12.9/share/pkgconfig:/opt/ros/kinetic/lib/pkgconfig pkg-config --libs ipopt`
##LIBS = -link -libpath:`$(CYGPATH_W) /home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib` libipopt.lib -llapack -lblas -lm  -ldl
#LIBS = -L/home/jom/Documents/prgs/projects/Ipopt-3.12.9/lib -lipopt -llapack -lblas -lm  -ldl

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: $(EXE)

.SUFFIXES: .cpp .c .o .obj

$(EXE): $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXLINKFLAGS) $(CXXFLAGS) -o $@ $$bla $(LIBS) $(ADDLIBS)

clean:
	rm -rf $(EXE) $(OBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<


.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
