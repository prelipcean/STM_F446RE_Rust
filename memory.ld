/* Entry Point: The program starts at reset_handler */
ENTRY(reset_handler)

/* Define memory regions for STM32F446RE */
MEMORY
{
  /* Flash memory: 512KB, used for code and constants */
  /* 0x0800 0000 - 0x081F FFFF */
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K

  /* SRAM1: 112KB, main RAM for data, stack, and heap */
  /* 0x2000 0000 - 0x2001 BFFF */
  SRAM1 (rwx)     : ORIGIN = 0x20000000, LENGTH = 112K

  /* SRAM2: 16KB, additional RAM (not used by default) */
  /* 0x2001 C000 - 0x2001 FFFF */
  SRAM2 (rw)     : ORIGIN = 0x2001C000, LENGTH = 16K

  /* Backup SRAM: 4KB, battery-backed RAM for persistent data */
  /* 0x4002 4000 - 0x4002 4FFF */
  BKPSRAM (rw)    : ORIGIN = 0x40024000, LENGTH = 4K
}

/* Stack pointer starts at the top of SRAM1 */
_start_of_stack = ORIGIN(SRAM1) + LENGTH(SRAM1) - 0x4;

/* Minimum stack and heap sizes (1KB each) */
_min_stack_size = 0x400;
_min_heap_size  = 0x400;

/*
 * SECTIONS command tells the linker where to place each section in memory.
 * The output ELF file will have these sections at the specified addresses.
 */
SECTIONS
{
    /* 
     * .text section: 
     * - Contains all executable code and interrupt vectors.
     * - Placed in FLASH.
     */
    .text :
    {
      . = ALIGN(4);                /* Align to 4 bytes */
      LONG(_start_of_stack);       /* Initial stack pointer value for vector table */
      KEEP(*(.isr_vector));        /* Interrupt vector table */
      *(.text)                     /* Main code */
      *(.text*)                    /* Any additional code sections */
      . = ALIGN(4);
    } > FLASH

    /*
     * .rodata section:
     * - Read-only data (constants, string literals).
     * - Placed in FLASH.
     */
    .rodata : 
    {
      . = ALIGN(4);
      *(.rodata)
      *(.rodata*)
      . = ALIGN(4);
    } > FLASH

    /*
     * .data section:
     * - Initialized global/static variables.
     * - Stored in FLASH, copied to SRAM1 at startup.
     * - AT> FLASH means load from FLASH, run from SRAM1.
     */
    .data :
    {
      _sidata = LOADADDR(.data);   /* Address of data in FLASH */
      . = ALIGN(4);
      _sdata = .;                  /* Start of .data in SRAM1 */
      *(.data)
      *(.data*)
      . = ALIGN(4);
      _edata = .;                  /* End of .data */
    } > SRAM1 AT> FLASH

    /*
     * .bss section:
     * - Uninitialized global/static variables (zero-initialized).
     * - Only exists in SRAM1 at runtime.
     */
    .bss :
    {
      . = ALIGN(4);
      _sbss = .;                   /* Start of .bss */
      *(.bss)
      *(.bss*)
      . = ALIGN(4);
      _ebss = .;                   /* End of .bss */
    } > SRAM1

    /*
     * .ram_usage_check section:
     * - Dummy section to reserve space for stack and heap.
     * - Ensures linker error if SRAM1 is over-allocated.
     */
    .ram_usage_check :
    {
      . = ALIGN(8);
      . = . + _min_stack_size;     /* Reserve stack */
      . = . + _min_heap_size;      /* Reserve heap */
      . = ALIGN(8);
    } > SRAM1

    /*
     * .noinit section:
     * - Variables placed here will NOT be initialized or zeroed at startup.
     * - Useful for data that must persist across software resets.
     * - Placed in SRAM2.
     * Example use cases:
     * - In Rust, use #[link_section = ".noinit"] for such variables.
     * - In C, use __attribute__((section(".noinit"))) to place variables here.
     */
    .noinit (NOLOAD) :
    {
      . = ALIGN(4);
      _snoinit = .;                /* Start of .noinit */
      *(.noinit)
      *(.noinit*)
      . = ALIGN(4);
      _enoinit = .;                /* End of .noinit */
    } > SRAM2
}

INCLUDE "device_STM32F446.x"