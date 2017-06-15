# $Id: Makefile,v 1.19 2017/06/14 23:04:54 karn Exp karn $
INCLUDES=-I /opt/local/include
COPTS=-g -O2 -std=gnu11 -pthread -Wall -funsafe-math-optimizations 
CFLAGS=$(COPTS) $(INCLUDES)

all: bandplan.txt radio control funcube pcm_monitor

install: all
	install --target-directory=$(HOME)/bin/ radio control funcube pcm_monitor

clean:
	rm -f *.o radio control funcube pcm_monitor bandplan.txt libfcd.a

funcube: funcube.o gr.o libfcd.a
	$(CC) -g -o $@ $^ -lasound -lusb-1.0 -lpthread -lm

control: control.o modes.o
	$(CC) -g -o $@ $^ -lm

radio: main.o radio.o demod.o am.o fm.o ssb.o iq.o cam.o filter.o display.o modes.o audio.o bandplan.o misc.o
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lpthread -lncurses -lopus -lm

pcm_monitor: pcm_monitor.o
	$(CC) -g -o $@ $^ -lasound  -lpthread -lncurses -lopus -lm

libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $^
	ranlib $@

am.o: am.c dsp.h filter.h radio.h audio.h
audio.o: audio.c dsp.h audio.h rtp.h
cam.o: cam.c dsp.h filter.h radio.h audio.h
control.o: control.c command.h
demod.o: demod.c radio.h
display.o: display.c radio.h audio.h dsp.h filter.h bandplan.h
fcd.o: fcd.c fcd.h hidapi.h fcdhidcmd.h
filter.o: filter.c dsp.h filter.h
fm.o: fm.c dsp.h filter.h radio.h audio.h
funcube.o: funcube.c fcd.h fcdhidcmd.h hidapi.h sdr.h command.h dsp.h rtp.h
gr.o: gr.c sdr.h
hid-libusb.o: hid-libusb.c hidapi.h
main.o: main.c radio.h filter.h dsp.h audio.h command.h rtp.h
misc.o: misc.c
modes.o: modes.c command.h
pcm_monitor.o: pcm_monitor.c rtp.h dsp.h
radio.o: radio.c command.h radio.h filter.h dsp.h audio.h
ssb.o: ssb.c dsp.h filter.h radio.h audio.h
iq.o: iq.c dsp.h filter.h radio.h audio.h
bandplan.o: bandplan.c bandplan.h
