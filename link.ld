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
  /* NOTE: probe-rs doesn't consider .bss section data
           when calculatin the target memory layout */
  .bss : {
    *(.bss)
    *(.bss.*)
    *(COMMON)
  }

  /* Read-only data section */
  /* NOTE: probe-rs doesn't consider .rodata section data
           when calculatin the target memory layout */
  .rodata : {
    *(.rodata)
    *(.rodata.*)
  }

  /* Discard all other sections */
  /DISCARD/ : {
    *(*)
  }
}
