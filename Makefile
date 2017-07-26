# $Id: Makefile,v 1.44 2017/07/24 02:26:57 karn Exp karn $
INCLUDES=-I /opt/local/include
#COPTS=-g -O2 -std=gnu11 -pthread -Wall -funsafe-math-optimizations
COPTS=-g    -std=gnu11 -pthread -Wall -funsafe-math-optimizations 
CFLAGS=$(COPTS) $(INCLUDES)
BINDIR=/usr/local/bin
LIBDIR=/usr/local/share/ka9q-radio

all: bandplan.txt help.txt radio control funcube monitor iqrecord iqplay modulate wwvsim wwv.txt wwvh.txt


install: all
	install -D --target-directory=$(BINDIR) radio control funcube monitor iqrecord iqplay modulate wwvsim
	install -D --target-directory=$(LIBDIR) bandplan.txt help.txt wwv.txt wwvh.txt

clean:
	rm -f *.o radio control funcube monitor iqrecord iqplay modulate wwvsim bandplan.txt help.txt wwv.txt wwvh.txt libfcd.a
	rcsclean

wwvsim: wwvsim.o
	$(CC) -g -o $@ $^ -lm

modulate: modulate.o misc.o filter.o modes.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lpthread -lm

funcube: funcube.o gr.o libfcd.a
	$(CC) -g -o $@ $^ -lasound -lusb-1.0 -lpthread -lm

iqplay: iqplay.o multicast.o misc.o
	$(CC) -g -o $@ $^ -lpthread -lm

control: control.o modes.o
	$(CC) -g -o $@ $^ -lm

radio: main.o radio.o demod.o am.o fm.o ssb.o iq.o cam.o dsb.o filter.o display.o modes.o audio.o multicast.o bandplan.o misc.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lpthread -lncurses -lopus -lm

monitor: monitor.o
	$(CC) -g -o $@ $^ -lasound  -lpthread -lopus -lm

osx_monitor: osx_monitor.o
	$(CC) -g -o $@ $^ -lopus -lm

iqrecord: iqrecord.o multicast.o
	$(CC) -g -o $@ $^ -lpthread -lm

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

am.o: am.c dsp.h filter.h radio.h audio.h
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
funcube.o: funcube.c fcd.h fcdhidcmd.h hidapi.h sdr.h dsp.h rtp.h radio.h
modulate.o: modulate.c dsp.h filter.h
gr.o: gr.c sdr.h
hid-libusb.o: hid-libusb.c hidapi.h
iq.o: iq.c dsp.h filter.h radio.h audio.h
iqplay.o: iqplay.c rtp.h dsp.h radio.h multicast.h
iqrecord.o: iqrecord.c rtp.h radio.h multicast.h
main.o: main.c radio.h filter.h dsp.h audio.h rtp.h multicast.h
misc.o: misc.c
modes.o: modes.c 
monitor.o: monitor.c rtp.h dsp.h
multicast.o: multicast.c multicast.h
radio.o: radio.c radio.h filter.h dsp.h audio.h
ssb.o: ssb.c dsp.h filter.h radio.h audio.h
wwvsim.o: wwvsim.c

