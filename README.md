# A tiny example program on STM32G031J6 with libopencm3

A tiny example on STM32G031J6 with LM75 compatible I2C tempreture sensor.

This program will generate a 23.4kHz PWM signal on pin4 according to the difference of tempretures measured by LM75 and STM32G031. Written for a simple fan control.

delay.[ch], stts75.[ch], rules.md and cortex-m-generic.ld are copied from libopencm3.

## Build

```
$ make
```

OPENCM3_DIR in Makefile should be set to your libopencm3 directory.


## Tools for stm32g031

ATM (Feb.2020), openocd and st-link don't support STM32G031 yet and a few changes are required. There are forked stm32g031 branches for [openocd](https://github.com/kazkojima/openocd) and [stlink](https://github.com/kazkojima/stlink).

## Write flash and option bytes

Connect stlink-v2 or something to stm32g031j6'swd pins (SWDCLK pin8, SWDIO pin7) and run openocd
```
$ openocd -f interface/stlink.cfg -f target/stm32g0x.cfg
```
and telnet to it from another terminal
```
$ telnet localhost 4444
```
then issue commands via telnet, for example
```
> reset halt
> flash write_image erase path_of_elf_binary
> exit
```

Change NRST_MODE field of option bytes from legacy to GPIO mode. (RM sec.3.4.1)
```
$ st-flash --debug --reset --format binary --flash=32k read dump.bin 0x1FFF7800 
4
(Binary editing dump.bin from aa fe ff ff to aa fe ff f7.)
$ st-flash --debug --reset --format binary --flash=32k write dump.bin 0x1FFF7800
```

Without changing NRST_MODE, any low signal on pin4 will reset CPU.
