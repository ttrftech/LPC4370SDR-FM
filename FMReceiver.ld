/* from FMReceiver_Release_Flash2RAM_lib.ld */

 GROUP(
 libcr_semihost.a
 libcr_c.a
 libcr_eabihelpers.a
 )


/* from FMReceiver_Release_Flash2RAM_mem.ld */

MEMORY
{
  /* Define each memory region */
  Flash_1MB (rx) : ORIGIN = 0x14000000, LENGTH = 0x100000 /* 1M bytes */
  RamLoc128 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x20000 /* 128K bytes */
  RamLoc72 (rwx) : ORIGIN = 0x10080000, LENGTH = 0x12000 /* 72K bytes */
}
  /* Define a symbol for the top of each memory region */
  __top_Flash_1MB = 0x14000000 + 0x100000;
  __top_RamLoc128 = 0x10000000 + 0x20000;
  __top_RamLoc72 = 0x10080000 + 0x12000;


/* from FMReceiver_Release_Flash2RAM.ld */

ENTRY(ResetISR)

SECTIONS
{

    /* MAIN TEXT SECTION */    
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.text_RAM));
        LONG(    ADDR(.text_RAM));
        LONG(  SIZEOF(.text_RAM));
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */
        
        *(.after_vectors*)
        
    } >Flash_1MB
    
    .text_RAM : ALIGN(4)    
    {
        . = ALIGN(4);
       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
        
    } > RamLoc128 AT>Flash_1MB

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    } > Flash_1MB
    __exidx_start = .;
    
    .ARM.exidx : ALIGN(4)
    {
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > Flash_1MB
    __exidx_end = .;
    
    _etext = .;
        
    
    /* DATA section for RamLoc72 */
    .data_RAM2 : ALIGN(4)
    {
       FILL(0xff)
       *(.ramfunc.$RAM2)
       *(.ramfunc.$RamLoc72)
    	*(.data.$RAM2*)
    	*(.data.$RamLoc72*)
       . = ALIGN(4) ;
    } > RamLoc72 AT>Flash_1MB
    
    /* MAIN DATA SECTION */
    

    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc128

	
	/* Main DATA section (RamLoc128) */
	.data : ALIGN(4)
	{
	   FILL(0xff)
	   _data = . ;
	   *(vtable)
	   *(.ramfunc*)
	   *(.data*)
	   . = ALIGN(4) ;
	   _edata = . ;
	} > RamLoc128 AT>Flash_1MB

    /* BSS section for RamLoc72 */
    .bss_RAM2 : ALIGN(4)
    {
    	*(.bss.$RAM2*)
    	*(.bss.$RamLoc72*)
       . = ALIGN(4) ;
    } > RamLoc72

    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RamLoc128
        
    /* NOINIT section for RamLoc72 */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
    	*(.noinit.$RAM2*)
    	*(.noinit.$RamLoc72*)
       . = ALIGN(4) ;
    } > RamLoc72 
    
    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc128
    
    PROVIDE(_pvHeapStart = DEFINED(__user_heap_base) ? __user_heap_base : .);
    PROVIDE(_vStackTop = DEFINED(__user_stack_top) ? __user_stack_top : __top_RamLoc128 - 0);
}
