INCLUDE_DIRS =
LIB_DIRS =
CC=g++

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS) -std=c++11
LIBS= -lrt
CPPLIBS=-L/usr/local/lib -lopencv_core -lopencv_flann -lopencv_video -lopencv_calib3d -lopencv_highgui

HFILES=
CFILES=
CPPFILES= drive.cpp #tpanl.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	drive #tpanl

clean:
	-rm -f *.o *.d cvtest*.ppm cvtest*.pgm test*.ppm test*.pgm
	-rm -f drive util

distclean:
	-rm -f *.o *.d

drive: drive.o util.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^  -lpthread -lrt `pkg-config --libs opencv` $(CPPLIBS) 

#tpanl: tpanl.o
#	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
