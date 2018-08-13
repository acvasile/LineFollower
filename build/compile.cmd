avr-g++ -mmcu=atmega324p -DF_CPU=16000000 -Wall -Os -o line_follower.elf ..\src\line_follower.c ..\src\usart.c

avr-objcopy -j .text -j .data -O ihex line_follower.elf line_follower.hex

avr-size line_follower.elf

.\bootloadHID -r line_follower.hex