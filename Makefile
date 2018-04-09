# $Id: Makefile,v 1.87 2018/04/09 21:26:25 karn Exp karn $
#CC=g++
INCLUDES=
#COPTS=-g -O2 -DNDEBUG=1 -std=gnu11 -pthread -Wall -funsafe-math-optimizations
COPTS=-g -std=gnu11 -pthread -Wall -funsafe-math-optimizations
CFLAGS=$(COPTS) $(INCLUDES)
BINDIR=/usr/local/bin
LIBDIR=/usr/local/share/ka9q-radio
EXECS=aprs funcube iqplay iqrecord modulate monitor opus opussend packet radio
AFILES=bandplan.txt help.txt modes.txt


all: $(EXECS) $(AFILES)

install: all
	install -o root -m 04755 -D --target-directory=$(BINDIR) $(EXECS)
	install -D --target-directory=$(LIBDIR) $(AFILES)

clean:
	rm -f *.o *.a $(EXECS) $(AFILES)
	rcsclean

aprs: aprs.o ax25.o multicast.o misc.o
	$(CC) -g -o $@ $^ -lbsd -lm

packet: packet.o multicast.o filter.o misc.o ax25.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lbsd -lm -lpthread 

opus: opus.o multicast.o
	$(CC) -g -o $@ $^ -lopus -lbsd -lm -lpthread

opussend: opussend.o multicast.o
	$(CC) -g -o $@ $^ -lopus -lportaudio -lbsd -lm -lpthread $(LD_FLAGS)

control: control.o modes.o
	$(CC) -g -o $@ $^ -lbsd -lm

funcube: funcube.o multicast.o gr.o libfcd.a
	$(CC) -g -o $@ $^ -lasound -lusb-1.0 -lbsd -lm -lpthread

iqplay: iqplay.o multicast.o attr.o misc.o
	$(CC) -g -o $@ $^ -lbsd -lm -lpthread

iqrecord: iqrecord.o multicast.o attr.o
	$(CC) -g -o $@ $^ -lbsd -lm -lpthread

modulate: modulate.o misc.o filter.o 
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lbsd -lm -lpthread

monitor: monitor.o multicast.o
	$(CC) -g -o $@ $^ -lopus -lportaudio -lbsd -lncurses -lm -lpthread

radio: main.o radio.o doppler.o fm.o am.o linear.o filter.o display.o modes.o audio.o multicast.o bandplan.o misc.o knob.o touch.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lncurses -lbsd -lm -lpthread

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

am.o: misc.h filter.h radio.h audio.h sdr.h
aprs.o: aprs.c ax25.h multicast.h misc.h
attr.o: attr.c attr.h
audio.o: audio.c misc.h audio.h multicast.h
ax25.o: ax25.c ax25.h
bandplan.o: bandplan.c bandplan.h
display.o: display.c radio.h sdr.h audio.h misc.h filter.h bandplan.h multicast.h
doppler.o: doppler.c radio.h sdr.h misc.h
fcd.o: fcd.c fcd.h hidapi.h fcdhidcmd.h
filter.o: filter.c misc.h filter.h
fm.o: fm.c misc.h filter.h radio.h sdr.h audio.h
funcube.o: funcube.c fcd.h fcdhidcmd.h hidapi.h sdr.h radio.h misc.h multicast.h
iqplay.o: iqplay.c misc.h radio.h sdr.h multicast.h attr.h
iqrecord.o: iqrecord.c radio.h sdr.h multicast.h attr.h
linear.o: linear.c misc.h filter.h radio.h sdr.h audio.h
main.o: main.c radio.h sdr.h filter.h misc.h audio.h multicast.h
misc.o: misc.c radio.h sdr.h
modes.o: modes.c radio.h sdr.h misc.h
modulate.o: modulate.c misc.h filter.h radio.h sdr.h
monitor.o: monitor.c misc.h multicast.h
multicast.o: multicast.c multicast.h
opus.o: opus.c misc.h multicast.h
opussend.o: opussend.c misc.h multicast.h
packet.o: packet.c filter.h misc.h multicast.h ax25.h
radio.o: radio.c radio.h sdr.h filter.h misc.h audio.h 

