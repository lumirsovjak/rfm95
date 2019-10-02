/*******************************************************************************
 * 
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>

#include <sys/ioctl.h>
#include <net/if.h>

using namespace std;

#include "base64.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

//TADY ZADEJ IP ADRESU NA CLIMATIX
#define IP "192.168.1.182"

typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

byte currentMode = 0x81;

char message[256];
char b64[256];

bool sx1272 = true;

byte receivedbytes;

struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
struct ifreq ifr;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF9;

// Set center frequency
uint32_t  freq = 868100000; // in Mhz! (868.1)

// Set location
float lat=48.0;
float lon=19.0;
int   alt=100;


// define servers
// TODO: use host names and dns
#define SERVER1 "127.0.0.1"    // The Things Network: croft.thethings.girovito.nl
//#define SERVER2 "192.168.1.10"      // local
#define PORT 1700                   // The port on which to send data

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

void die(const char *s)
{
    perror(s);
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}


boolean receivePkt(char *payload)
{

    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);

    cp_nb_rx_rcv++;

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        cp_nb_rx_ok++;

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

	printf("-----------------------------\n");
	printf("payload \n\n");

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
	    printf("%d",payload[i]);
	    printf(" ");
        }
	printf("\n\n");
	printf("teplota=");
	printf("%c",payload[4]);
	printf("%c",payload[5]);
	printf("%c",payload[6]);
	printf("\n\n");

//teplota 2x za sebou 456 789, adresa 3x za sebou 10 11, 12 13, 14 15, baterie 16 17 18 19 kdy 16 bude vzdy nula 

	string prikazR1 = ""; //reliabilita R, teplota T, Baterie B 
	string prikazR2 = "";
	string prikazR3 = "";
	string prikazT1 = "";
	string prikazT2 = "";
	string prikazT3 = "";
	string prikazB1 = "";
	string prikazB2 = "";
	string prikazB3 = "";

	prikazR1 = prikazR1 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Reliability1;"; //kontrola toku dat
	prikazR2 = prikazR2 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Reliability2;"; //kontrola toku dat
	prikazR3 = prikazR3 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Reliability3;"; //kontrola toku dat
	prikazR1 = prikazR1 + "1"; //do kontrolnich bodu zapisuje jednicky, climatix je nuluje
	prikazR2 = prikazR2 + "1"; //do kontrolnich bodu zapisuje jednicky, climatix je nuluje
	prikazR3 = prikazR3 + "1"; //do kontrolnich bodu zapisuje jednicky, climatix je nuluje

	prikazT1 = prikazT1 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Termostat1;";
	prikazT2 = prikazT2 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Termostat2;";
	prikazT3 = prikazT3 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Termostat3;";
	prikazT1 = prikazT1 + payload[4] + payload[5] + payload[6];
	prikazT2 = prikazT2 + payload[4] + payload[5] + payload[6];
	prikazT3 = prikazT3 + payload[4] + payload[5] + payload[6];
	
	prikazB1 = prikazB1 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Baterie1;"; //kontrola napeti
	prikazB2 = prikazB2 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Baterie2;"; //kontrola napeti
	prikazB3 = prikazB3 + "curl --user ADMIN:SBTAdmin! \"http://" + IP + "/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_Baterie3;"; //kontrola napeti
	
	//prevod payload na char, protoze teplota je char rovnou a baterky ne, mozna predelat i v arduinu? taky baterka na char
	payload[17] = 48 + payload[17];
	payload[18] = 48 + payload[18];
	payload[19] = 48 + payload[19];
	prikazB1 = prikazB1 + payload[17] + payload[18] + payload[19];
	prikazB2 = prikazB2 + payload[17] + payload[18] + payload[19];
	prikazB3 = prikazB3 + payload[17] + payload[18] + payload[19];

	prikazR1 = prikazR1 + "\"";
	prikazR2 = prikazR2 + "\"";
	prikazR3 = prikazR3 + "\"";
	prikazT1 = prikazT1 + "\"";
	prikazT2 = prikazT2 + "\"";
	prikazT3 = prikazT3 + "\"";
	prikazB1 = prikazB1 + "\"";
	prikazB2 = prikazB2 + "\"";
	prikazB3 = prikazB3 + "\"";

	if ((payload[11]==1) && (payload[13]==1) && (payload[15]==1)) std::cout << prikazT1 << endl; 
	if ((payload[11]==2) && (payload[13]==2) && (payload[15]==2)) std::cout << prikazT2 << endl; 
	if ((payload[11]==3) && (payload[13]==3) && (payload[15]==3)) std::cout << prikazT3 << endl; 

	const char *commandR1 = prikazR1.c_str();
	const char *commandR2 = prikazR2.c_str();
	const char *commandR3 = prikazR3.c_str();
	const char *commandT1 = prikazT1.c_str();
	const char *commandT2 = prikazT2.c_str();
	const char *commandT3 = prikazT3.c_str();
	const char *commandB1 = prikazB1.c_str();
	const char *commandB2 = prikazB2.c_str();
	const char *commandB3 = prikazB3.c_str();

	if ((payload[11]==1) && (payload[13]==1) && (payload[15]==1)) system(commandR1);
	if ((payload[11]==2) && (payload[13]==2) && (payload[15]==2)) system(commandR2);
	if ((payload[11]==3) && (payload[13]==3) && (payload[15]==3)) system(commandR3);

	if ((payload[11]==1) && (payload[13]==1) && (payload[15]==1)) system(commandT1);
	if ((payload[11]==2) && (payload[13]==2) && (payload[15]==2)) system(commandT2);
	if ((payload[11]==3) && (payload[13]==3) && (payload[15]==3)) system(commandT3);

	if ((payload[11]==1) && (payload[13]==1) && (payload[15]==1)) system(commandB1);
	if ((payload[11]==2) && (payload[13]==2) && (payload[15]==2)) system(commandB2);
	if ((payload[11]==3) && (payload[13]==3) && (payload[15]==3)) system(commandB3);

	printf("\n");

	//system("curl --user ADMIN:SBTAdmin! \"http://192.168.3.16/JSON.HTML?FN=Write&PIN=1000&LNG=1&ID=T_TermostatJSON;456\"");
	printf("\n");
    }
	
    return true;
}

void SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x12); // LoRaWAN public sync word 0x34 moej je default 0x12

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
        } else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);//zapnuty  LNA AGC loop
        }
        writeRegister(REG_MODEM_CONFIG,0x78);// bylo 0x72, moje je 0x78 tj 125kHz a cr4/8
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD,0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

}



void receivepacket() {

    long int SNR;
    int rssicorr;

    if(digitalRead(dio0) == 1)
    {
        if(receivePkt(message)) {
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ",readRegister(0x1A)-rssicorr);
            printf("RSSI: %d, ",readRegister(0x1B)-rssicorr);
            printf("SNR: %li, ",SNR);
            printf("Length: %i",(int)receivedbytes);
            printf("\n");



            fflush(stdout);

        } // received a message

    } // dio0=1
}

int main () {

    struct timeval nowtime;
    uint32_t lasttime;

    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    //int fd = 
    wiringPiSPISetup(CHANNEL, 500000);
    //cout << "Init result: " << fd << endl;

    SetupLoRa();

    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);  // can we rely on eth0?
    ioctl(s, SIOCGIFHWADDR, &ifr);

    /* display result */
    printf("Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
           (unsigned char)ifr.ifr_hwaddr.sa_data[0],
           (unsigned char)ifr.ifr_hwaddr.sa_data[1],
           (unsigned char)ifr.ifr_hwaddr.sa_data[2],
           (unsigned char)ifr.ifr_hwaddr.sa_data[3],
           (unsigned char)ifr.ifr_hwaddr.sa_data[4],
           (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

    printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
    printf("------------------\n");

    while(1) {

        receivepacket();

        gettimeofday(&nowtime, NULL);
        uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
        if (nowseconds - lasttime >= 30) {
            lasttime = nowseconds;
            //sendstat();
            cp_nb_rx_rcv = 0;
            cp_nb_rx_ok = 0;
            cp_up_pkt_fwd = 0;
        }
        delay(1);
    }

    return (0);

}

