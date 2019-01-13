# $Id: Makefile,v 1.136 2019/01/08 06:21:32 karn Exp karn $
COPTS=-g -DNDEBUG=1 -O3 -march=native -std=gnu11 -pthread -Wall -funsafe-math-optimizations
#COPTS=-g -march=native -std=gnu11 -pthread -Wall -funsafe-math-optimizations
CFLAGS=$(COPTS) $(INCLUDES)
BINDIR=/usr/local/bin
LIBDIR=/usr/local/share/ka9q-radio
LDLIBS=-lpthread -lbsd -lm
EXECS=aprs aprsfeed funcube hackrf iqplay iqrecord modulate monitor opus opussend packet pcmsend radio pcmcat control metadump pl
AFILES=bandplan.txt help.txt modes.txt
SYSTEMD_FILES=funcube0.service funcube1.service hackrf0.service radio34.service radio39.service packet.service aprsfeed.service opus-hf.service opus-vhf.service opus-hackrf.service opus-uhf.service
UDEV_FILES=66-hackrf.rules 68-funcube-dongle-proplus.rules 68-funcube-dongle.rules 69-funcube-ka9q.rules

all: $(EXECS) $(AFILES) $(SYSTEMD_FILES) $(UDEV_FILES)

install: $(EXECS) $(AFILES)
	install -o root -m 0755 -D --target-directory=$(BINDIR) $(EXECS)
	install -D --target-directory=$(LIBDIR) $(AFILES)

systemd: $(SYSTEMD_FILES) $(UDEV_FILES)
	install -o root -m 0644 -D --target-directory=/etc/systemd/system $(SYSTEMD_FILES)
	systemctl daemon-reload
	install -o root -m 0644 -D --target-directory=/etc/udev/rules.d $(UDEV_FILES)
	adduser --system aprsfeed
	adduser --system funcube
	adduser --system hackrf

clean:
	rm -f *.o *.a $(EXECS)

.PHONY: clean all install

# Executables
aprs: aprs.o ax25.o libradio.a
aprsfeed: aprsfeed.o libradio.a
control: control.o modes.o misc.o multicast.o bandplan.o status.o
	$(CC) -g -o $@ $^ -lncurses -lbsd -lpthread -lm

funcube: funcube.o libradio.a libfcd.a
	$(CC) -g -o $@ $^ -lportaudio -lusb-1.0 -lbsd -lm -lpthread

hackrf: hackrf.o libradio.a
	$(CC) -g -o $@ $^ -lhackrf -lbsd -lpthread -lm

iqplay: iqplay.o libradio.a
iqrecord: iqrecord.o libradio.a
metadump: metadump.o multicast.o status.o libradio.a
	$(CC) -g -o $@ $^ -lbsd -lpthread -lm

modulate: modulate.o libradio.a
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lm

monitor: monitor.o libradio.a
	$(CC) -g -o $@ $^ -lopus -lportaudio -lncurses -lbsd -lm -lpthread

opus: opus.o libradio.a
	$(CC) -g -o $@ $^ -lopus -lbsd -lm -lpthread

opussend: opussend.o libradio.a
	$(CC) -g -o $@ $^ -lopus -lportaudio -lbsd -lm

packet: packet.o ax25.o libradio.a
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lbsd -lm -lpthread

pcmcat: pcmcat.o libradio.a
	$(CC) -g -o $@ $^ -lm -lbsd -lpthread 

pcmsend: pcmsend.o libradio.a
	$(CC) -g -o $@ $^ -lportaudio -lbsd

pl: pl.o libradio.a
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lm -lpthread    

radio: main.o audio.o fm.o linear.o modes.o radio.o radio_status.o status.o libradio.a
	$(CC) -g -o $@ $^ -lfftw3f_threads -lfftw3f -lncurses -lbsd -lpthread -lm

# Binary libraries
libfcd.a: fcd.o hid-libusb.o
	ar rv $@ $?
	ranlib $@

libradio.a: attr.o ax25.o decimate.o dsp.o filter.o misc.o multicast.o rtcp.o status.o osc.o dump.o
	ar rv $@ $?
	ranlib $@

# Main programs
aprs.o: aprs.c ax25.h multicast.h misc.h dsp.h
aprsfeed.o: aprsfeed.c ax25.h multicast.h misc.h
control.o: control.c radio.h osc.h sdr.h  misc.h filter.h bandplan.h multicast.h dsp.h status.h
funcube.o: funcube.c fcd.h fcdhidcmd.h hidapi.h sdr.h misc.h multicast.h status.h dsp.h
hackrf.o: hackrf.c sdr.h misc.h multicast.h decimate.h status.h dsp.h
iqplay.o: iqplay.c misc.h radio.h osc.h sdr.h multicast.h attr.h
iqrecord.o: iqrecord.c radio.h osc.h sdr.h multicast.h attr.h
metadump.o: metadump.c multicast.h dsp.h status.h misc.h
modulate.o: modulate.c misc.h filter.h radio.h osc.h sdr.h
monitor.o: monitor.c misc.h multicast.h
opus.o: opus.c misc.h multicast.h
opussend.o: opussend.c misc.h multicast.h
packet.o: packet.c filter.h misc.h multicast.h ax25.h dsp.h osc.h
pcmcat.o: pcmcat.c multicast.h
pcmsend.o: pcmsend.c misc.h multicast.h
pl.o: pl.c misc.h multicast.h dsp.h


# Components of libfcd.a
fcd.o: fcd.c fcd.h hidapi.h fcdhidcmd.h
hid-libusb.o: hid-libusb.c hidapi.h

# components of libradio.a
attr.o: attr.c attr.h
ax25.o: ax25.c ax25.h
decimate.o: decimate.c decimate.h
dsp.o: dsp.c dsp.h misc.h
dump.o: dump.c misc.h status.h
filter.o: filter.c misc.h filter.h dsp.h
knob.o: knob.c misc.h
misc.o: misc.c misc.h 
multicast.o: multicast.c multicast.h misc.h
rtcp.o: rtcp.c multicast.h
status.o: status.c status.h misc.h
touch.o: touch.c misc.h
osc.o: osc.c  osc.h


# Components of radio
am.o: am.c misc.h filter.h radio.h osc.h sdr.h dsp.h multicast.h
audio.o: audio.c misc.h  multicast.h
bandplan.o: bandplan.c bandplan.h
display.o: display.c radio.h osc.h sdr.h  misc.h filter.h bandplan.h multicast.h dsp.h
doppler.o: doppler.c radio.h osc.h sdr.h misc.h
fm.o: fm.c misc.h filter.h radio.h osc.h sdr.h dsp.h multicast.h
linear.o: linear.c misc.h filter.h radio.h osc.h sdr.h dsp.h multicast.h
main.o: main.c radio.h osc.h sdr.h filter.h misc.h  multicast.h dsp.h status.h
misc.o: misc.c radio.h osc.h sdr.h
modes.o: modes.c radio.h sdr.h osc.h misc.h
radio.o: radio.c radio.h sdr.h osc.h filter.h misc.h
radio_status.o: radio_status.c status.h radio.h misc.h dsp.h filter.h multicast.h sdr.h






