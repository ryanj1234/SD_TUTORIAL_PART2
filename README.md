# SD_TUTORIAL_PART2

This is the source code for the tutorial located here:
http://rjhcoding.com/avrc-sd-interface-2.php

Edit the makefile and change MCU to the microcontroller you are using, and AVRDUDE_PROGRAMMER to your programmer.

Type make to build. Expected output:

```
Compiling: main.c
avr-gcc -c -mmcu=atmega328p -I. -DF_CPU=16000000UL  -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=main.lst  -std=gnu99 main.c -o main.o

Linking: main.elf
avr-gcc -mmcu=atmega328p -I. -DF_CPU=16000000UL  -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=main.o  -std=gnu99 main.o --output main.elf -Wl,-Map=main.map,--cref

Creating load file for Flash: main.hex
avr-objcopy -O ihex -R .eeprom main.elf main.hex

Creating load file for EEPROM: main.eep
avr-objcopy -j .eeprom --set-section-flags .eeprom=alloc,load \
--change-section-lma .eeprom=0 -O ihex main.elf main.eep
avr-objcopy: --change-section-lma .eeprom=0x0000000000000000 never used
```

type make program to write to device:
