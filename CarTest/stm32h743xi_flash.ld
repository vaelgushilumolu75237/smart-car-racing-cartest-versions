/*
******************************************************************************
**
** @brief       : Linker script for STM32H743xI devices from STM32H7 series
**                      2048KBytes FLASH
**                      512KBytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2026 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Memories definition */
MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  /* Build-specific RAM logger layout descriptor consumed by the offline
     HotPlug exporter. Keep its symbol and bytes in the selected ELF. */
  .runtime_log_descriptor : ALIGN(4)
  {
    KEEP(*(.runtime_log_descriptor))
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH
  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data section */
  .data : ALIGN(4)
  {
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */
    . = ALIGN(4);
  } >RAM_D1 AT> FLASH

  .tdata : ALIGN(4)
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
    PROVIDE(__data_end = .);
    PROVIDE(__tdata_end = .);
  } >RAM_D1 AT> FLASH
  PROVIDE( __tdata_start = ADDR(.tdata) );
  PROVIDE( __tdata_size = __tdata_end - __tdata_start );

  PROVIDE( __data_start = ADDR(.data) );
  PROVIDE( __data_size = __data_end - __data_start );

  PROVIDE( __tdata_source = LOADADDR(.tdata) );
  PROVIDE( __tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata) );
  PROVIDE( __tdata_source_size = __tdata_source_end - __tdata_source );

  PROVIDE( __data_source = LOADADDR(.data) );
  PROVIDE( __data_source_end = __tdata_source_end );
  PROVIDE( __data_source_size = __data_source_end - __data_source );

  /* Uninitialized data section */
  .tbss (NOLOAD) : ALIGN(4)
  {
    /* This is used by the startup in order to initialize the .bss and .tbss sections */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    PROVIDE( __bss_start = . );
    *(.tbss .tbss.*)
    . = ALIGN(4);
    PROVIDE( __tbss_end = . );
  } >RAM_D1
  PROVIDE( __tbss_start = ADDR(.tbss) );
  PROVIDE( __tbss_size = __tbss_end - __tbss_start );
  PROVIDE( __tbss_offset = ADDR(.tbss) - ADDR(.tdata) );

  PROVIDE( __tls_base = __tdata_start );
  PROVIDE( __tls_end = __tbss_end );
  PROVIDE( __tls_size = __tls_end - __tls_base );
  PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
  PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1) );
  PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );
  PROVIDE( __arm64_tls_tcb_offset = MAX(16, __tls_align) );

  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
    PROVIDE( __bss_end = . );
  } >RAM_D1
  PROVIDE( __bss_size = __bss_end - __bss_start );
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );

  /* Independent SWD-readable CPU snapshot. It has its own 64-byte ABI and is
     kept separate from the fixed-size V1/V2 runtime-log allocation. */
  .runtime_diag (NOLOAD) : ALIGN(32)
  {
    __runtime_diag_start = .;
    KEEP(*(.runtime_diag))
    . = ALIGN(32);
    __runtime_diag_end = .;
  } >RAM_D1

  ASSERT((ADDR(.runtime_diag) & 31) == 0,
         "runtime diagnostic section is not 32-byte aligned")
  ASSERT(SIZEOF(.runtime_diag) == 64,
         "runtime diagnostic ABI must reserve exactly 64 bytes")

  .receive_diag (NOLOAD) : ALIGN(32)
  {
    __receive_diag_start = .;
    KEEP(*(.receive_diag))
    . = ALIGN(32);
    __receive_diag_end = .;
  } >RAM_D1

  ASSERT((ADDR(.receive_diag) & 31) == 0,
         "receive diagnostic section is not 32-byte aligned")
  ASSERT(SIZEOF(.receive_diag) == 96,
         "receive diagnostic ABI must reserve exactly 96 bytes")

  /* Independent encoder speed-PI diagnostic snapshot. Keep it separate from
     both the CPU snapshot and the fixed-size runtime log for SWD inspection. */
  .speed_pi_diag (NOLOAD) : ALIGN(32)
  {
    __speed_pi_diag_start = .;
    KEEP(*(.speed_pi_diag))
    . = ALIGN(32);
    __speed_pi_diag_end = .;
  } >RAM_D1

  ASSERT((ADDR(.speed_pi_diag) & 31) == 0,
         "speed PI diagnostic section is not 32-byte aligned")
  ASSERT(SIZEOF(.speed_pi_diag) == 128,
         "speed PI diagnostic ABI must reserve exactly 128 bytes")

  .speed_pi_trace (NOLOAD) : ALIGN(32)
  {
    __speed_pi_trace_start = .;
    KEEP(*(.speed_pi_trace))
    . = ALIGN(32);
    __speed_pi_trace_end = .;
  } >RAM_D2

  ASSERT((ADDR(.speed_pi_trace) & 31) == 0,
         "speed PI trace section is not 32-byte aligned")
  ASSERT(SIZEOF(.speed_pi_trace) == 98368,
         "speed PI trace ABI must reserve exactly 98368 bytes")
  ASSERT(ADDR(.speed_pi_trace) >= 0x30000000 &&
         ADDR(.speed_pi_trace) + SIZEOF(.speed_pi_trace) <= 0x30020000,
         "speed PI trace must fit in the 128 KiB clocked D2 SRAM1 bank")

  /* The log is SWD-readable, NOLOAD, cache-line aligned, and not initialized
     or cleared with .bss. Runtime code initializes only its 64-byte header. */
  .runtime_log (NOLOAD) : ALIGN(32)
  {
    __runtime_log_start = .;
    KEEP(*(.runtime_log))
    . = ALIGN(32);
    __runtime_log_end = .;
  } >RAM_D1

  ASSERT((ADDR(.runtime_log) & 31) == 0,
         "runtime log section is not 32-byte aligned")
  ASSERT(SIZEOF(.runtime_log) == 393280,
         "runtime log ABI must reserve exactly 393280 bytes")
  ASSERT((_estack - __runtime_log_end) >= 32768,
         "runtime log leaves less than 32 KiB RAM_D1 stack/heap headroom")

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  ASSERT(SIZEOF(._user_heap_stack) >= _Min_Heap_Size + _Min_Stack_Size,
         "RAM_D1 heap and stack reservation is incomplete")

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a:* ( * )
    libm.a:* ( * )
    libgcc.a:* ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
