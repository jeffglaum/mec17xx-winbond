SECTIONS
{
  /* Start of memory */
  . = 0xc0020;

  /* API Code section */
  .api : {
    *(.api)
    *(.api.*)
  }

  /* Code section */
  .text : {
    *(.text)
    *(.text.*)
  }

  /* Initialized data section */
  .data : {
    *(.data)
    *(.data.*)
  }

  /* Uninitialized data section */
  .bss : {
    *(.bss)
    *(.bss.*)
    *(COMMON)
  }

  /* Read-only data section */
  .rodata : {
    *(.rodata)
    *(.rodata.*)
  }

  /* Discard all other sections */
  /DISCARD/ : {
    *(*)
  }
}
