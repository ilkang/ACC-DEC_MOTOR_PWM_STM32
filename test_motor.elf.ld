
/*
!!! Primary linker script file generated automatically !!!
This file contains the input and output files names
It also includes the secondary/main linker script that is specified in the Ride options (default or custom)
*/

SEARCH_DIR(".")
SEARCH_DIR("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest")
SEARCH_DIR("D:\Program Files (x86)\Raisonance\Ride\Lib\ARM")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\test_motor.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\startup_stm32f10x_md.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\system_stm32f10x.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\stm32f10x_rcc.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\stm32f10x_gpio.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\stm32f10x_exti.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\stm32f10x_tim.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\stm32f10x_usart.o")
INPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\misc.o")
EXTERN( __io_putchar )
EXTERN( _write )
GROUP(
"STM32x_io_putchar_thumb.a"
"std_sbrk_thumb.lib"
"e_stdio_thumb.a"
"STM32F10x_thumb.lib"
"libc.a"
"libm.a"

)
INCLUDE "D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\STM32F103_64K_20K_FLASH.ld"
OUTPUT("D:\Dropbox\Work\MWIR\Firmware\[5]MWIR - DRV8825_2 - Acctest\test_motor.elf")
