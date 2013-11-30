# location of the Python header files

PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)

# location of the Boost Python include files and library
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

# compile mesh classes
TARGET = cvxgen_controller


$(TARGET).so: $(TARGET).o
	g++ -shared -Wl,--export-dynamic $(TARGET).o util.o matrix_support.o solver.o ldl.o -L$(BOOST_LIB) -lboost_python-py27 -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION) -o $(TARGET).so

$(TARGET).o: $(TARGET).c
	g++ -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -fPIC -DBOOST_PYTHON_MAX_ARITY=40 -c $(TARGET).c util.c matrix_support.c solver.c ldl.c

clean:
	rm *.o