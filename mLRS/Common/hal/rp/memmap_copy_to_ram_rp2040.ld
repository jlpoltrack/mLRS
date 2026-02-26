/* copy_to_ram linker script for rp2040
 * based on framework memmap_default.ld, modified so all code goes to ram
 * the standard crt0 data copy handles loading .data (which now includes .text)
 * from flash into sram at boot — no PICO_COPY_TO_RAM flag needed
 * 2026-02-06
 */

MEMORY
{
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 2044k
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 256k
    SCRATCH_X(rwx) : ORIGIN = 0x20040000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20041000, LENGTH = 4k
}

/* eeprom at last 4k of 2mb flash, no filesystem */
PROVIDE ( _EEPROM_start = 0x101FF000 );
PROVIDE ( _FS_start     = 0x101FF000 );
PROVIDE ( _FS_end       = 0x101FF000 );

ENTRY(_entry_point)

SECTIONS
{
    /* second stage bootloader, must be 256 bytes */

    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    .boot2 : {
        __boot2_start__ = .;
        KEEP (*(.boot2))
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ == 256,
        "ERROR: Pico second stage bootloader must be 256 bytes in size")

    .ota : {
        KEEP (*(.OTA))
    } > FLASH

    .partition : {
        . = __flash_binary_start + 0x2ff0;
        LONG(0x101FF000)
        LONG(0x101FF000)
        LONG(0x101FF000)
        LONG(2093056)
    } > FLASH

    /* only vectors, binary_info, and reset handler stay in flash */

    .text : {
        __logical_binary_start = .;
        KEEP (*(.vectors))
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.embedded_block))
        __embedded_block_end = .;
        KEEP (*(.reset))
    } > FLASH

    .rodata : {
        /* segments marked .flashdata stay in flash */
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    . = ALIGN(4);

    /* vector table goes first in ram to avoid large alignment hole */
    .ram_vector_table (NOLOAD): {
        *(.ram_vector_table)
    } > RAM

    /* all code and data is placed in .data, which crt0 copies from flash to ram.
     * this works because the standard startup code copies everything between
     * __etext (flash load address) and __data_start__ through __data_end__ (ram). */
    .data : {
        __data_start__ = .;
        *(vtable)

        *(.time_critical*)

        /* all program code — copied to ram at boot */
        *(.init)
        *(.text*)
        *(.fini)
        /* c'tors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* d'tors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.eh_frame*)

        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)

        . = ALIGN(4);
        *(.after_data.*)

        . = ALIGN(4);
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.jcr)
        . = ALIGN(4);
    } > RAM AT> FLASH

    .tdata : {
        . = ALIGN(4);
		*(.tdata .tdata.* .gnu.linkonce.td.*)
        __tdata_end = .;
    } > RAM AT> FLASH
    PROVIDE(__data_end__ = .);

    __etext = LOADADDR(.data);

    .tbss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start__ = .;
        __tls_base = .;
        *(.tbss .tbss.* .gnu.linkonce.tb.*)
        *(.tcommon)

        __tls_end = .;
    } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __tbss_end = .;

        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    /* at most one of the following two will be engaged */
    .uninitialized_ram MAX(0x20003000, .) (NOLOAD) : {
        *(.uninitialized_ram*)
    } > RAM

    .uninitialized_data MAX(0x20003000, .) (NOLOAD) : {
        *(.uninitialized_data*)
    } > RAM

    .heap (NOLOAD):
    {
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        . = ORIGIN(RAM) + LENGTH(RAM);
        __HeapLimit = .;
    } > RAM

    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH
    __scratch_y_source__ = LOADADDR(.scratch_y);

    .stack1_dummy (NOLOAD):
    {
        *(.stack1*)
    } > SCRATCH_X
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > SCRATCH_Y

    .flash_end : {
        KEEP(*(.embedded_end_block*))
        PROVIDE(__flash_binary_end = .);
    } > FLASH

    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackOneTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    PROVIDE (__heap_start = __end__);
    PROVIDE (__heap_end = __HeapLimit);
    PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
    PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1));
    PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );

    PROVIDE (_end = __end__);
    PROVIDE (__llvm_libc_heap_limit = __HeapLimit);

    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")
    ASSERT( __binary_info_header_end - __logical_binary_start <= 256, "Binary info must be in first 256 bytes of the binary")
}
