CC = g++
CFLAGS = -Wall
PROG = Arachnophobia

SRCS = Arachnophobia.cpp
LIBS = -lopenal -lalut -lglut -lGL -lGLU

all: $(PROG)

$(PROG):	$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

clean:
	rm -f $(PROG)
