# RPI-GPIO-DS1921L
Code for interfacing with a DS1921L iButton Thermochron from userspace on a Raspberry Pi
Angular Fish

## What?
The Maxim (formerly Dallas Semiconductor) DS1921L thermocron is an iButton datalogger popular among ecologists. They are cheap (~10 - ~30 USD), small, and relatively hassle-free. Communication uses DS/Maxim's 1-wire interface and is powered parasitically, a small lithium primary battery powers the temperature conversion circuitry, and temperature datalogging and device registers are contained in on-chip battery-backed SRAM. To spec, the battery should last 10 years and then I guess it's destined for the fuckit bucket (ROM commands still work after the battery dies, but reading the serial number is a trick that has very little use to me. You might be able to register it as a bus token in Istanbul, but I'm not sure that's useful here in North America). The datasheet can be found at: https://datasheets.maximintegrated.com/en/ds/DS1921L-F5X.pdf. Precision is 0.5 °C. They are not waterproof but that is easy enough to fix.

This software contains functions to read and write to the DS1921L from the GPIO pins on a Raspberry Pi. It is incomplete, must be run as root, and many functions are currently untested, and I'm a fish biologist not a programmer, so YMMV. At present the main function just runs a loop to do single-shot temperature conversions every five minutes and cough them up on stdout. There's more here though so feel free to use it. The world's your oyster.

## Why?
I was given some dead iButtons by another researcher years ago. They have festered in a suitcase gathering dust for a long while and I recently decided to see if battery replacement was a viable activity. It is. The batteries are nominally 3v lithium batteries, but they also seem to survive at the 3.3v that the RPI puts out. I was able to use them fairly reliably with an Arduino, but the linux 1-wire kernel module used on the Raspberry Pi is a confusing and frustrating thing.

I am currently trying to stratify some seeds at 4°C for 60 days and have a beer fridge which hasn't been used much in recent years. I used one of these salvaged iButtons and this code to monitor the internal temperature of the refrigerator to see if it's appropriate. I'm just doing repeated single-shot conversions and diverting stdout to a text file on a shared drive. So far it's looking like a snowball has a better chance of surviving a close solar orbit than this refrigerator has at staying near 4°C for two months.

## How?
1-wire is pretty well-documented so I won't get into it very much here. For more information check out the datasheet and tutorial at iotprogrammer.com: https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/36-raspberry-pi-and-the-iot-in-c-one-wire-basics. Communication is slow, timing is non-critical (close counts in horseshoes and hand grenades and 1-wire, apparently), and timing between bits is, as near as I can tell, completely ignored.

Device specific communication adds a layer of complexity. The basic operation is:
1. Reset the device
2. Execute a ROM command (read/confirm/skip the device's serial number, good for having multiple devices on the same bus)
3. Execute a RAM command or start a conversion
4. Wait for some time
5. Reset again

Registers are not manipulated directly, but saved to a scratchpad and then confirmed and committed in separate reset cycles, so this chip is going to be reset a lot.

To connect the device to your RPI, you will need a wire from the outside case to the PI's ground, and a second wire from the upper case to a GPIO pin. You will also need to connect the 3.3v pin to the upper case through a 2.2 kΩ resistor. If, like me, you are using a board scavenged from a dead iButton, you will also need to connect the positive battery terminal on the board to the Pi's 3.3v rail.

To compile the code you will need the BCM2835 library which can be found here:
https://www.airspayce.com/mikem/bcm2835/

gcc -o ibutton -Wall ibutton.cc -l bcm2835
