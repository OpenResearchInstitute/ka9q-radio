INCLUDES=-I ../fcd -I /opt/local/include
COPTS=-g -std=gnu11 -pthread -Wall -D_GNU_SOURCE=1 -D_REENTRANT=1  -funsafe-math-optimizations 
CFLAGS=$(COPTS) $(INCLUDES)

all: radio control funcube

clean:
	rm -f *.o radio control funcube

funcube: funcube.o gr.o ../libfcd.a
	$(CC) -g -o $@ $^ ../libfcd.a -lasound -lusb-1.0 -lpthread -lm

control: control.o modes.o
	$(CC) -g -o $@ $^ -lm

radio: main.o radio.o demod.o fm.o filter.o display.o modes.o audio.o misc.o
	$(CC) -g -o $@ $^ -lasound  -lfftw3f_threads -lfftw3f -lpthread -lm

main.o: main.c sdr.h radio.h audio.h
demod.o: demod.c dsp.h radio.h fm.h audio.h
filter.o: filter.c dsp.h
radio.o: radio.c sdr.h radio.h audio.h
display.o: display.c sdr.h radio.h audio.h
audio.o: audio.c sdr.h radio.h dsp.h audio.h
fm.o: fm.c dsp.h radio.h  fm.h
misc.o: misc.c
funcube.o: funcube.c




