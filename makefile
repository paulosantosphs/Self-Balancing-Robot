

#avr-gcc -mmcu=atmega328p -O2 -Wall -o main.elf main.c
#avr-objcopy -O ihex main.elf main.hex

#sudo /home/yifan/Desktop/arduino-1.0.5/hardware/tools/avrdude -C/home/yifan/Desktop/arduino-1.0.5/hardware/tools/avrdude.conf -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:`pwd`/main.hex:i
		


# define useful variables
MMCU = atmega328p
PORT = /dev/ttyACM0
BAUD = 115200

NAME = main

# define important constants
F_CPU = 16000000UL
BAUD_UART = 9600


# important paths
PATH_AVR_GCC = /usr/lib/avr/bin

AVRDUDE_CONF = /etc/avrdude.conf
AVRDUDE = /usr/bin/avrdude


# gcc & avrdude flags 
GCC_FLAG = -mmcu=$(MMCU) -Os -DF_CPU=$(F_CPU) -DBAUD=$(BAUD_UART) 
AVRDUDE_FLAG = -v -p $(MMCU) -c arduino -P $(PORT) -b $(BAUD) 


all:	main.hex

main.hex: main.elf
	avr-objcopy -O ihex main.elf main.hex

main.elf: *.c
	avr-gcc $(GCC_FLAG) -o main.elf *.c

upload:	main.hex
	avrdude $(AVRDUDE_FLAG) -F -Uflash:w:$(NAME).hex:i





