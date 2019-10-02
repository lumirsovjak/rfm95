# single_chan_pkt_fwd
# Single Channel LoRaWAN Gateway
#zkusime upravit, prejmenujeme na rfm95 a budeme spoustet ./rfm95
#smazal jsem clean: a co bylo za nim

CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: single_chan_pkt_fwd

single_chan_pkt_fwd: base64.o main.o
	$(CC) main.o base64.o $(LIBS) -o rfm95

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

base64.o: base64.c
	$(CC) $(CFLAGS) base64.c

clean:
	rm *.o single_chan_pkt_fwd
