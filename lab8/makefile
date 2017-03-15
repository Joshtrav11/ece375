CC=avr-g++
CXX=.cpp
MAIN=MAINFILE
ISP=usbasp 
SOURCES=$(MAIN)
OBJECTS=$(SOURCES:$(CXX)=.o) 
ELFS=$(MAIN:$(CXX)=.elf)
HEX=$(MAIN:$(CXX)=.hex)
HEXASM=$(MAIN:.asm=.hex)
override LDFLAGS= -Wl,-u,vfprintf -lprintf_flt -lm
######## COMPILING ########

teensy: BOARD=atmega32u4
teensy: SOURCES+=adc.cpp
teensy: SOURCES+=spi/spi32u4.cpp
teensy: SOURCES+=led/led32u4.cpp
teensy: compile
	teensy_loader_cli -mmcu=atmega32u4 -w $(HEX)

arduino2: arduino
	avrdude -c arduino -P /dev/tty.usbmodem1421 -p m328p -U flash:w:$(HEX)

arduino1: arduino
	avrdude -c arduino -P /dev/tty.usbmodem1411 -p m328p -U flash:w:$(HEX)
	
arduino: BOARD=atmega328p
arduino: SOURCES+=uart.cpp
arduino: SOURCES+=adc.cpp
arduino: SOURCES+=./spi/spi328p.cpp
arduino: SOURCES+=./led/led328p.cpp
arduino: compile

asm: BOARD=atmega128
asm:
	gavrasm $(MAIN)
	echo $(HEXASM)
	avrdude -c $(ISP) -P usb -p m128 -U flash:w:$(HEXASM)

128: BOARD=atmega128
128: SOURCES+=spi/spi128.cpp
128: SOURCES+=led/led128.cpp
128: SOURCES+=m128def.inc
128: SOURCES+=RF24.cpp
128: compile
	avrdude -c $(ISP) -P usb -p m128 -U flash:w:$(HEX)

85: BOARD=attiny85
85: SOURCES+=./spi/spi85.cpp
85: SOURCES+=./RF24.cpp
85: SOURCES+=./led/led85.cpp
85: compile
	avrdude -v -c osuisp2 -P usb -p t85 -U flash:w:$(HEX)

84: BOARD=attiny84
84: SOURCES+=./spi/spi84.cpp
84: SOURCES+=./led/led84.cpp
84: compile
	avrdude -v -c osuisp2 -P usb -p t84 -U flash:w:$(HEX)

328: BOARD=atmega328
328: SOURCES+=./led/led328.cpp
328: compile
	avrdude -v -c osuisp2 -P usb -p m328p -F -U flash:w:$(HEX)

compile:
	$(CC) $(LDFLAGS) -g -Os -mmcu=$(BOARD) -c $(SOURCES)
	mv led*.o ./led/
	mv spi*.o ./spi/
	$(CC) $(LDFLAGS) -g -mmcu=$(BOARD) -o $(ELFS) $(OBJECTS)
	avr-objcopy -v -j .text -j .data -O ihex $(ELFS) $(HEX)


clean:
	rm -f *.lst *.o *.elf
