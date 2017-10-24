# $Id: Makefile,v 1.74 2017/10/20 22:33:31 karn Exp karn $
#CC=g++
INCLUDES=
COPTS=-g -O2 -DNDEBUG=1 -std=gnu11 -pthread -Wall -funsafe-math-optimizations
#COPTS=-g -std=gnu11 -pthread -Wall -funsafe-math-optimizations
CFLAGS=$(COPTS) $(INCLUDES)
BINDIR=/usr/local/bin
LIBDIR=/usr/local/share/ka9q-radio

all: funcube iqplay iqrecord modulate monitor radio bandplan.txt help.txt modes.txt

install: all
	install -o root -m 04755 -D --target-directory=$(BINDIR) radio funcube monitor iqrecord iqplay modulate
	install -D --target-directory=$(LIBDIR) bandplan.txt help.txt modes.txt

clean:
	rm -f *.o *.a control funcube iqplay iqrecord modulate monitor radio
	rcsclean

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
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lncurses -lopus -lbsd -lportaudio -lm -lpthread

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

am.o: am.c dsp.h filter.h radio.h audio.h
attr.o: attr.c attr.h
audio.o: audio.c dsp.h audio.h rtp.h
bandplan.o: bandplan.c bandplan.h
control.o: control.c dsp.h radio.h
display.o: display.c radio.h audio.h dsp.h filter.h bandplan.h multicast.h rtp.h
knob.o: knob.c dsp.h
touch.o: touch.c dsp.h
doppler.o: doppler.c radio.h filter.h dsp.h audio.h
fcd.o: fcd.c fcd.h hidapi.h fcdhidcmd.h
filter.o: filter.c dsp.h filter.h
fm.o: fm.c dsp.h filter.h radio.h audio.h
funcube.o: funcube.c fcd.h fcdhidcmd.h hidapi.h sdr.h radio.h dsp.h rtp.h multicast.h
gr.o: gr.c sdr.h
hid-libusb.o: hid-libusb.c hidapi.h
iqplay.o: iqplay.c rtp.h dsp.h radio.h multicast.h attr.h
iqrecord.o: iqrecord.c rtp.h radio.h multicast.h attr.h
linear.o: linear.c dsp.h filter.h radio.h audio.h
main.o: main.c radio.h filter.h dsp.h audio.h rtp.h multicast.h
misc.o: misc.c
modes.o: modes.c 
modulate.o: modulate.c dsp.h filter.h radio.h
monitor.o: monitor.c rtp.h dsp.h multicast.h
multicast.o: multicast.c multicast.h
radio.o: radio.c radio.h filter.h dsp.h audio.h
