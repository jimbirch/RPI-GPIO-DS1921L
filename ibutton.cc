/* Userspace code for interfacing with the Dallas Semiconductor/Maxim DS1921L
 * "iButton" thermochron
 *
 * 2021 Angular Fish
 
 * Includes code for bitbanging the 1-wire protocol and some DS1921L-specific
 * functions cribbed from the datasheet. Currently loops through a single
 * conversion to measure the temperature inside my refrigerator, but
 * you can use the other functions at you liesure. So far these include:
 * Single-shot conversion
 * Set the RTC from the Pi's time
 * Start a mssion

 * Still in development:
 * Read a mission

 * Bit banging routines based on IoT Programmer's tutorial at:
 * https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/36-raspberry-pi-and-the-iot-in-c-one-wire-basics,
 * and Paul Stoffregren's arduino 1-wire library (https://www.pjrc.com/teensy/td_libs_OneWire.html)

 * There are some nice kernel modules for 1-wire devices, particularly
 * DS/Maxim's temperature sensors, but I have not had much luck using them
 * to communicate with the DS1921L iButtons that I have replaced the batteris
 * in. Luckily, 1-wire is pretty easy to bitbang from userspace on the Pi's
 * GPIO. These devices (at least the communication part) are powered
 * parasitically from the communication's line and can be reset by pulling the
 * line low for long enough to draw down their capacitor. They respond when the
 * line comes high again with a low pulse of their own after a short time.

 * Bits are sent by initiating a "frame" by pulling the line low and then
 * keeping the line low or pushing it high after a few microseconds. The
 * device will sample after this time and register a 1 if the line is high
 * or 0 if the line is low.

 * Bits are received by initiating a "frame" by pulling the line low briefly
 * and then letting it float high. The device will hold the line low for a
 * few microseconds and then hold it low for most of the frame to send a 0
 * or let it float high to send a 1. The Pi needs to sample at this period
 * to receive the bit.

 * Commands sent to the device must follow this format:
 * 1. Reset (and confirm presence pulse)
 * 2. Send a ROM command (Read or confirm the device ID)
 * 3. Send a RAM command (Read or write the RAM)
 * 3a. (Or send a 1-shot conversion command)
 * 4. Do nothing else

 * The device gets reset a lot. To do a 1 shot conversion, for example:
 * 1. Reset
 * 2. Skip ROM
 * 3. Send 1-shot conversion command
 * 4. Wait out the conversion time
 * 5. Reset
 * 6. Skip ROM
 * 7. Send read memory command
 * 8. Send address for single conversion
 * 9. Read 1 byte

 * It gets worse from here. To start a mission for example:
 * 1. Reset
 * 2. Skip ROM
 * 3. Send write memory command
 * 4. Send address for RTC registers
 * 5. Send time (7 bytes)
 * 6. Reset
 * 7. Skip ROM
 * 8. Send read scratchpad command
 * 9. Read address for RTC registers
 * 10. Read end offset
 * 11. Confirm bytes
 * 12. Reset
 * 13. Skip ROM
 * 14. Send commit scratchpad command
 * 15. Send address for RTC registers
 * 16. Send end offset
 * 17. Wait for memory to commit
 * 18. Reset
 * 19. Skip ROM
 * 20. Send write memory command
 * 21. Send address for control register
 * 22. Send clear memory enable
 * 23. Reset
 * 24. Skip ROM
 * 25. Send read scratchpad command
 * 26. Read address
 * 27. Read end offset
 * 28. Confirm bytes
 * 29. Reset
 * 30. Skip ROM
 * 31. Send commit scratchpad command
 * 32. Send address for control register
 * 33. Send end offset
 * 34. Wait for memory to commit
 * 35. Reset
 * 36. Skip ROM
 * 37. Send write memory command
 * 38. Send address for control register
 * 39. Send control register for mission
 * 40. Send 3 empty bytes
 * 41. Send 16 bit start delay
 * 42. Reset
 * 43. Send read scratchpad
 * 44. Read address for control register
 * 45. Read address for end offset
 * 46. Confirm bytes
 * 47. Reset
 * 48. Skip ROM
 * 49. Send commit scratchpad
 * 50. Send address for control register
 * 51. Send end offset
 * 52. Wait for scratchpad to commit
 * 53. Reset

 * So there you have it, starting a mission in 53 easy steps! It gets even
 * worse if you need to deal with multiple devices.
 */


#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ROM Functions are the first functions to run
 * after reset
 */

#define READROM 0x33
#define MATCHROM 0x55
#define SEARCHROM 0xF0
#define SKIPROM 0xCC
#define CONDITIONALSEARCH 0xEC

/* RAM Functions run after completing ROM functions
 */
#define WRITESCRATCH 0x0F
#define READSCRATCH 0xAA
#define COPYSCRATCH 0x55
#define READMEM 0xF0
#define READMEMCRC 0xA5
#define CLEARMEM 0x3C

/* Single shot conversion isn't really a RAM function
 * but runs in the RAM function part of code
 */
#define CONVERTTEMP 0x44

/* Memory mapping for the chip */
#define SRAMSTART 0x0000
#define REGISTERSTART 0x0200
#define ALARMSTART 0x0220
#define RESERVED1 0x0280
#define HISTSTART 0x0800
#define RESERVED2 0x0880
#define DATALOGSTART 0x1000
#define RESERVED3 0x1800

/* Special addresses */
#define TEMPADDR 0x0211
#define RTCSECONDS 0x0200
#define RTCMINUTES 0x0201
#define RTCHOURS 0x0202
#define RTCDAYOFWEEK 0x203
#define RTCDATE 0x0204
#define RTCMONTH 0x0205
#define RTCYEAR 0x0206
#define RCTALARMSECS 0x0207
#define RTCALARMMINS 0x0208
#define RTCALARMHRS 0x0209
#define RTCALARMDOW 0x020A
#define CONTROLREG 0x020E
#define MISDELAY 0x0212

/* Control register bits */
#define ENABLEOSC 0b00000000
#define ENABLECLR 0b01000000
#define ENABLEMIS 0b00000000
#define ENABLERLO 0b00001000
#define ENABLETLS 0b00000100
#define ENABLETHS 0b00000010
#define ENABLETAS 0b00000001

/* Global definition for connection pin to
 * potentially be changed by command line args
 */
uint8_t targetpin = RPI_GPIO_P1_16;

/* 1-wire bit banging functions cobbled together
 * from the arduino 1-wire library:
 * https://www.pjrc.com/teensy/td_libs_OneWire.html
 * and "Super User"'s tutorial at:
 * https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/36-raspberry-pi-and-the-iot-in-c-one-wire-basics
 */
void writeBit(uint8_t pin, int b) {
	int delay1, delay2;
	if(b==1) {
		delay1 = 10;
		delay2 = 55;
	} else {
		delay1 = 65;
		delay2 = 5;
	}
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(pin, LOW);
	bcm2835_delayMicroseconds(delay1);
	bcm2835_gpio_write(pin, HIGH);
	bcm2835_delayMicroseconds(delay2);
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
}

void writeByte(uint8_t pin, int byte) {
	int i;
	for(i = 0; i < 8; i++) {
		if(byte & 1) {
			writeBit(pin, 1);
		} else {
			writeBit(pin, 0);
		}
		byte = byte >> 1;
	}
}

uint8_t readBit(uint8_t pin) {
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(pin, LOW);
	bcm2835_delayMicroseconds(5);
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
	bcm2835_delayMicroseconds(10);
	uint8_t b = bcm2835_gpio_lev(pin);
	bcm2835_delayMicroseconds(53);
	return b;
}

int readByte(uint8_t pin) {
	int byte = 0;
	int i;
	for(i = 0; i < 8; i++) {
		byte = byte | (readBit(pin) << i);
	};
	return byte;
}

int reset(uint8_t pin) {
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_write(pin, LOW);
	bcm2835_delayMicroseconds(480);
	bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
	bcm2835_delayMicroseconds(70);
	uint8_t b = bcm2835_gpio_lev(pin);
	bcm2835_delayMicroseconds(410);
	return b;
}

void delay(int seconds) {
	/* clock_t starttime = clock();
	uint32_t endtime = starttime + seconds * CLOCKS_PER_SEC;
	while(clock() < endtime) { }; */
	bcm2835_delayMicroseconds(seconds * 1000000);
}

/* DS1921L specific functions */
void writeAddr(uint8_t pin, int address) {
	uint8_t byte1 = (address >> 8) & 0xFF;
	uint8_t byte2 = address & 0xFF;
	writeByte(pin, byte2);
	writeByte(pin, byte1);
}

float oneShotConvert(uint8_t pin) {
	float temperature = 0;
	int check = 1;
	check = reset(pin);
	if(check == HIGH) return -100; // Returns an impossible temp
	writeByte(pin, SKIPROM);
	writeByte(pin, CONVERTTEMP);
	bcm2835_delayMicroseconds(200);

	check = reset(pin);
	if(check == HIGH) return -100;
	writeByte(pin, SKIPROM);
	writeByte(pin, READMEM);
	writeAddr(pin, TEMPADDR);
	temperature = readByte(pin) / 2.0 - 40.0;
	return temperature;
}

uint8_t BCDSeconds(time_t time) {
	uint8_t seconds;
	seconds = time % 60;
	return seconds;
}

bool verifyScratch(uint8_t pin, uint16_t address, uint8_t length) {
	reset(pin);
	writeByte(pin,SKIPROM);
	writeByte(pin,READSCRATCH);
	uint16_t returnaddress = (uint16_t)readByte(pin);
	returnaddress |= (uint16_t)readByte(pin) << 8;
	uint8_t returnlen = readByte(pin);
	uint8_t endoffset = address & 0x1F;
	endoffset += length - 1;
	if(returnaddress == address && returnlen == endoffset) {
		return true;
	} else {
		return false;
	}
}

void commitScratch(uint8_t pin, uint16_t address, uint8_t length) {
	uint8_t endoffset = address & 0x1F;
	endoffset += length - 1;
	reset(pin);
	writeByte(pin,SKIPROM);
	writeByte(pin,COPYSCRATCH);
	writeAddr(pin,address);
	writeByte(pin, endoffset);
	bcm2835_delayMicroseconds(100);
}

void setRTC(uint8_t pin) {
	struct tm * timestruct;
	time_t currtime;
	int check = 1;
	uint8_t lowbyte;
	uint8_t highbyte;
	uint8_t bcdbyte;
	check = reset(pin);
	if(check == HIGH) {
		printf("Error setting RTC.");
		return;
	}
	writeByte(pin, SKIPROM);
	writeByte(pin, WRITESCRATCH);
	writeAddr(pin, RTCSECONDS);
	
	time(&currtime);
	timestruct = localtime(&currtime);
	
	lowbyte = (timestruct->tm_sec % 10) & 0x0F;
	highbyte = (timestruct->tm_sec / 10) & 0x0F;
	bcdbyte = (highbyte << 4) | lowbyte;

	writeByte(pin, bcdbyte);

	lowbyte = (timestruct->tm_min % 10) & 0x0F;
	highbyte = (timestruct->tm_min / 10) & 0x0F;
	bcdbyte = (highbyte << 4) | lowbyte;

	writeByte(pin, bcdbyte);

	lowbyte = (timestruct->tm_hour % 10) & 0x0F;
	highbyte = ((uint8_t)(timestruct->tm_hour > 10)) | ((uint8_t)(timestruct->tm_hour > 20)) << 1 | 1 << 2;
	bcdbyte = (highbyte << 4) | lowbyte;

	writeByte(pin, bcdbyte);

	bcdbyte = (uint8_t)(timestruct->tm_wday + 1);
	writeByte(pin, bcdbyte);

	lowbyte = ((timestruct->tm_mon  + 1) % 10) & 0x0F;
	highbyte = ((timestruct->tm_mon + 1) / 10) & 0x0F;
	bcdbyte = 1 << 7 | (highbyte << 4) | lowbyte;
	writeByte(pin, bcdbyte);

	lowbyte = ((timestruct->tm_year - 100) % 10) & 0x0F;
	highbyte = ((timestruct->tm_year - 100) / 10) & 0x0F;
	bcdbyte = (highbyte << 4) | lowbyte;
	writeByte(pin, bcdbyte);

	if(verifyScratch(pin, RTCSECONDS, 7)) commitScratch(pin, RTCSECONDS, 7);
	else printf("Failed to set RTC\n");
}

void clearMem(uint8_t pin) {
	reset(pin);
	writeByte(pin,SKIPROM);
	writeByte(pin,WRITESCRATCH);
	writeAddr(pin,CONTROLREG);
	writeByte(pin,ENABLECLR);
	if(verifyScratch(pin, CONTROLREG, 1)) commitScratch(pin, CONTROLREG, 1);
	reset(pin);
	writeByte(pin,SKIPROM);
	writeByte(pin,CLEARMEM);
	reset(pin);
}

void missionStart(uint8_t pin, uint16_t delay, uint8_t creg) {
	reset(pin);
	writeByte(pin,SKIPROM);
	writeByte(pin,WRITESCRATCH);
	writeAddr(pin,CONTROLREG);
	writeByte(pin,creg);
	writeByte(pin,0x00); // 3 0s are sent to write through and save an extra scratchpad write
	writeByte(pin,0x00); // per the datasheet this should do nothing.
	writeByte(pin,0x00);
	writeAddr(pin,delay); // Start delay is a 16 bit integer stored in two locations
				 // writeAddress sends two 8 bit integers consecutively
	if(verifyScratch(pin, CONTROLREG, 6)) commitScratch(pin, CONTROLREG, 6);
	reset(pin);
}

int main(int argc, char *argv[]) {
	if(!bcm2835_init()) return 1;
	printf("time, id, temperature\n");
	time_t rawtime;
	while(true) {
		time(&rawtime);
		char* str1 = ctime(&rawtime);
		str1[strcspn(str1,"\n")] = 0;
		printf("%20s, ", str1);
		int check = 0;
		int ret = 0;
		float temp = 0;
		check = reset(targetpin);
		if(check == HIGH) printf("failed to connect.\n");
		writeByte(targetpin, 0x33);
		int i = 0;
		for(i = 0; i < 8; i++) {
			ret = readByte(targetpin);
			printf("%X",ret);
		}
		printf(", ");
		temp = oneShotConvert(targetpin); 
		/* writeByte(targetpin, 0x44);
		bcm2835_delayMicroseconds(300000);
		check = reset(targetpin);
		if(check == HIGH) printf("failed to reconnect.\n");
		writeByte(targetpin, 0xCC);
		writeByte(targetpin, 0xF0);
		writeByte(targetpin, 0x11);
		writeByte(targetpin, 0x02);
		ret = readByte(targetpin);
		temp = ((float)ret / 2) - 40;*/

		printf("%.1f\n",temp);
		fflush(stdout);
		delay(300);
	}
	return 0;
}
