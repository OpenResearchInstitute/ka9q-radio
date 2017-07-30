# $Id: Makefile,v 1.49 2017/07/29 10:17:57 karn Exp karn $
INCLUDES=-I /opt/local/include
COPTS=-g -O2 -std=gnu11 -pthread -Wall -funsafe-math-optimizations
CFLAGS=$(COPTS) $(INCLUDES)
BINDIR=/usr/local/bin
LIBDIR=/usr/local/share/ka9q-radio

all: control funcube iqplay iqrecord modulate monitor radio bandplan.txt help.txt 

install: all
	install -D --target-directory=$(BINDIR) radio control funcube monitor iqrecord iqplay modulate
	install -D --target-directory=$(LIBDIR) bandplan.txt help.txt

clean:
	rm -f *.o *.a control funcube iqplay iqrecord modulate monitor radio bandplan.txt help.txt 
	rcsclean

control: control.o modes.o
	$(CC) -g -o $@ $^ -lm

funcube: funcube.o gr.o libfcd.a
	$(CC) -g -o $@ $^ -lasound -lusb-1.0 -lpthread -lm

iqplay: iqplay.o multicast.o attr.o misc.o
	$(CC) -g -o $@ $^ -lpthread -lm

iqrecord: iqrecord.o multicast.o attr.o
	$(CC) -g -o $@ $^ -lpthread -lm

modulate: modulate.o misc.o filter.o modes.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lpthread -lm

monitor: monitor.o multicast.o
	$(CC) -g -o $@ $^ -lasound -lopus -lm

radio: main.o radio.o demod.o am.o fm.o ssb.o iq.o cam.o dsb.o filter.o display.o modes.o audio.o multicast.o bandplan.o misc.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lpthread -lncurses -lopus -lm

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

am.o: am.c dsp.h filter.h radio.h audio.h
attr.o: attr.c attr.h
audio.o: audio.c dsp.h audio.h rtp.h
bandplan.o: bandplan.c bandplan.h
cam.o: cam.c dsp.h filter.h radio.h audio.h
control.o: control.c dsp.h
demod.o: demod.c radio.h
display.o: display.c radio.h audio.h dsp.h filter.h bandplan.h multicast.h
dsb.o: dsb.c dsp.h filter.h radio.h audio.h
fcd.o: fcd.c fcd.h hidapi.h fcdhidcmd.h
filter.o: filter.c dsp.h filter.h
fm.o: fm.c dsp.h filter.h radio.h audio.h
gr.o: gr.c sdr.h
hid-libusb.o: hid-libusb.c hidapi.h
iq.o: iq.c dsp.h filter.h radio.h audio.h
iqplay.o: iqplay.c rtp.h dsp.h radio.h multicast.h attr.h
iqrecord.o: iqrecord.c rtp.h radio.h multicast.h attr.h
main.o: main.c radio.h filter.h dsp.h audio.h rtp.h multicast.h
misc.o: misc.c
modes.o: modes.c 
modulate.o: modulate.c dsp.h filter.h
monitor.o: monitor.c rtp.h dsp.h multicast.h
multicast.o: multicast.c multicast.h
radio.o: radio.c radio.h filter.h dsp.h audio.h
ssb.o: ssb.c dsp.h filter.h radio.h audio.h
