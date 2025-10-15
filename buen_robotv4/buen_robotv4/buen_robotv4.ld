/* buen_robotv4.ld - TM4C1294NCPDT */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00100000
  SRAM  (rwx): ORIGIN = 0x20000000, LENGTH = 0x00040000
}

SECTIONS
{
  .text :
  {
    _text = .;
    KEEP(*(.isr_vector))
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
    __etext = .;
  } > FLASH

  .data : AT(__etext)
  {
    _data  = .;
    _ldata = LOADADDR(.data);
    *(.data*)
    _edata = .;
  } > SRAM

  .bss :
  {
    _bss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > SRAM
}

