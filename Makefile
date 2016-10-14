# $Id: Makefile,v 1.8 2016/10/14 04:43:25 karn Exp karn $
INCLUDES=-I /opt/local/include
COPTS=-g -std=gnu11 -pthread -Wall -funsafe-math-optimizations 
CFLAGS=$(COPTS) $(INCLUDES)

all: radio control funcube

clean:
	rm -f *.o radio control funcube libfcd.a

funcube: funcube.o gr.o libfcd.a
	$(CC) -g -o $@ $^ libfcd.a -lasound -lusb-1.0 -lpthread -lm

control: control.o modes.o
	$(CC) -g -o $@ $^ -lm

radio: main.o radio.o demod.o fm.o filter.o display.o modes.o audio.o misc.o
	$(CC) -g -o $@ $^ -lasound  -lfftw3f_threads -lfftw3f -lpthread -lm

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

control.o: control.c command.h
main.o: main.c radio.h filter.h dsp.h audio.h command.h
demod.o: demod.c dsp.h filter.h radio.h fm.h audio.h command.h
filter.o: filter.c dsp.h filter.h
radio.o: radio.c command.h radio.h filter.h dsp.h audio.h
display.o: display.c radio.h audio.h sdr.h dsp.h
audio.o: audio.c dsp.h audio.h
fm.o: fm.c dsp.h radio.h fm.h audio.h
misc.o: misc.c
funcube.o: funcube.c fcd.h fcdhidcmd.h sdr.h command.h dsp.h
fcd.o: fcd.c hidapi.h fcd.h fcdhidcmd.h
hid-libusb.o: hid-libusb.c hidapi.h





