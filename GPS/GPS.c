/*
 * GPS.c
 *
 *  Created on: Mar 4, 2017
 *      Author: Eason Smith
 *
 *      The GPS class processes incoming binary GPS messages received via USART1
 */

#include "usart/usart.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"


#define START1 0xa0
#define START2 0xa1
#define TRAILER1 0x0d
#define TRAILER2 0x0a

#define MAXPAYLOADLEN 200
#define NAVLEN 59

typedef struct NAVstruct { //payload of 59 bytes
   uint8_t id; //message type
   uint8_t fixmode;
   uint8_t numsv;
   uint16_t gnss_week_num;
   uint32_t gnss_time_of_week; //
   int32_t latitude; //in decdeg aka mult by 1x10^-7
   int32_t longitude;
   uint32_t altitude_ellip;
   uint32_t altitude_sealvl; //height above sealevel
   uint16_t gdop;
   uint16_t pdop;
   uint16_t hdop;
   uint16_t vdop;
   uint16_t tdop;
   int32_t ecef_x;
   int32_t ecef_y;
   int32_t ecef_z;
   int32_t ecef_velocity_x; //speed
   int32_t ecef_velocity_y;
   int32_t ecef_velocity_z;
} NAVstruct;

typedef union GPS {
	uint8_t raw[59];
	NAVstruct current;
} GPS;
static GPS NAVdata;  //now can do NAVdata.current.latitude etc.

typedef struct GPSmsg {//todo: add timestamp
   uint16_t payload_length;
   uint8_t payload[MAXPAYLOADLEN];
   uint8_t end[2];
   uint8_t checksum;
} GPSmsg;
static GPSmsg msg;

//GPS State Machine
typedef enum {GETSTART, GETLENGTH, GETPAYLOAD, GETCHECKSUM, GETTRAILER, VALIDATE} GPS_STATE;
static GPS_STATE state = GETSTART;

//todo: wait on get fix? wait on pps? capture time?
void GPS_init(void) {
    // state=INITIALIZING
	// state=WARMINGUP
	// todo: insert wait for 1min30secs cold start. while(usart <1) {timeout if more than 2 mins? throw error}
	// state=RECEVINGDATA
	// done. call process GPS or have process GPS call GPS_init the first time through and set a flag
}

//void GPS_debug(char* s, SemaphoreHandle_t* m){

//process incoming USART1 messages into valid GPS data
void processGPS(void){
	switch(state){
	case GETSTART:
		//wait for 2 bytes over usart
		while(usart1_available() > 1){
				uint8_t received1 = usart1_read();
				uint8_t received2 = usart1_read();

				//printf("%x %x ", received1, received2);
				if(received1 == START1 && received2 == START2){
					state = GETLENGTH;
				}
				else if (received1 != START1)
				{
				    printf("\ns1 fail: %x\n", received1);
					break;
				}
				else if (received2 != START2)
				{
					printf("\ns2 fail: %x\n", received2);
					break;
				}
    	}
		break;
	case GETLENGTH:
		//wait 2 bytes, cast to uint16 length
		//wait for 2 bytes over usart
		while(usart1_available() > 1){
				uint16_t highbyte = usart1_read() << 8;
				uint16_t lowbyte = usart1_read();
				uint16_t len = highbyte | lowbyte;
				//printf("%x ", highbyte);
				//printf("%x ", lowbyte);
				msg.payload_length = len;
				state = GETPAYLOAD;
				}
		break;
	case GETPAYLOAD:
		//wait for length number of bytes
		if(msg.payload_length > MAXPAYLOADLEN) {
			printf("Err:Payload arr maxed!: %d", msg.payload_length);
			state = GETSTART;
			break;
		}
		while(usart1_available() > (msg.payload_length-1)){
			for(uint16_t i = 0;i < msg.payload_length;i++){
				msg.payload[i] = usart1_read();
				//printf("%x ", msg.payload[i]);
			}
			state = GETCHECKSUM;
		}
		break;
	case GETCHECKSUM:
		//wait for 1 byte
		while(usart1_available() > 0){
			msg.checksum = usart1_read();
			//printf("[%x] ", msg.checksum);
			state=GETTRAILER;
		}
		break;
	case GETTRAILER:
		//wait for 2 bytes
		while(usart1_available() > 1){
				uint8_t trailer_rec1 = usart1_read();
				uint8_t trailer_rec2 = usart1_read();
				printf("%x %x ", trailer_rec1, trailer_rec2);

				if (trailer_rec1 != TRAILER1)
				{
					printf("\ntr1 fail: %x\n", trailer_rec1);
					state = GETSTART;
					break;
				}
				else if (trailer_rec2 != TRAILER2)
				{
					printf("\ntr2 fail: %x\n", trailer_rec2);
					state = GETSTART;
					break;
				}
				state = VALIDATE;
				break;//exit while loop
		}
		break;
	case VALIDATE:
		//todo:timestamp
		//todo:uint8_t result = validatePayload(msg.payload_length,msg.payload,msg.checksum);

		//calc checksum
		if(msg.payload_length > MAXPAYLOADLEN) { printf("validate error!"); state = GETSTART; break; }
		uint8_t total = 0;
		for(int i = 0;i < msg.payload_length;i++) {
			total ^= msg.payload[i];
		}

		printf("len: %d, chk: %x, mychk: %x \n", msg.payload_length,msg.checksum,total, total);

		if(total == msg.checksum) {
			printf("PAYLOAD GOOD!\n");
		}

		//if ok cast payload to appropriate navmsg
		//reset state to -->getstart
		state = GETSTART;
		break;
	}
}

uint8_t validateChecksum(uint8_t payload_length, uint8_t* payload, uint8_t checksum){
	//if ((payload_length == 0) || (payload == 0)) {
	//	return 0;
	//}

	//calc checksum
	uint8_t total = 0;
	for(int i = 0;i < payload_length;i++) {
		total ^= payload[i];
	}

	if(checksum == total) {
		return 1;
	}
	return 0;
}

/*
//todo:
#define NAVID
void HandleGPSPayload(uint8_t msg[MAXPAYLOADLEN], uint8_t msglen){
uint8_t IDBYTE = msg[0]; //0x01 up to 0xFF
uint32_t LEN = msglen;//up to 65535 bytes
switch(IDBYTE){
	case NAVDATA:
		// transferred in little-endian format. low byte first followed by highbyte.
		//wait for 2 bytes over usart

		break;
	case default:
		//handle unknown message type;
}*/
