/* Linker script for STM32L0xx6, 32K flash, 8K RAM. */

/* Define memory regions. */
MEMORY
{
        rom (rx) : ORIGIN = 0x08000000, LENGTH = 32K
        ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld
