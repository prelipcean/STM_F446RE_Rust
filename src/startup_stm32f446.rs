use core::ptr;

unsafe extern "C" {
    unsafe fn BusFault_Handler();
    unsafe fn MemManage_Handler();
    unsafe fn PendSV_Handler();
    unsafe fn SVCall_Handler();
    unsafe fn DebugMon_Handler();
    unsafe fn SysTick_Handler();
    unsafe fn UsageFault_Handler();
    unsafe fn ADC_Handler();
    unsafe fn CAN1_RX0_Handler();
    unsafe fn CAN1_RX1_Handler();
    unsafe fn CAN1_SCE_Handler();
    unsafe fn CAN1_TX_Handler();
    unsafe fn CAN2_RX0_Handler();
    unsafe fn CAN2_RX1_Handler();
    unsafe fn CAN2_SCE_Handler();
    unsafe fn CAN2_TX_Handler();
    unsafe fn DCMI_Handler();
    unsafe fn FPU_Handler();
    unsafe fn DMA1_Stream0_Handler();
    unsafe fn DMA1_Stream1_Handler();
    unsafe fn DMA1_Stream2_Handler();
    unsafe fn DMA1_Stream3_Handler();
    unsafe fn DMA1_Stream4_Handler();
    unsafe fn DMA1_Stream5_Handler();
    unsafe fn DMA1_Stream6_Handler();
    unsafe fn DMA1_Stream7_Handler();
    unsafe fn DMA2_Stream0_Handler();
    unsafe fn DMA2_Stream1_Handler();
    unsafe fn DMA2_Stream2_Handler();
    unsafe fn DMA2_Stream3_Handler();
    unsafe fn DMA2_Stream4_Handler();
    unsafe fn DMA2_Stream5_Handler();
    unsafe fn DMA2_Stream6_Handler();
    unsafe fn DMA2_Stream7_Handler();
    unsafe fn EXTI0_Handler();
    unsafe fn EXTI15_10_Handler();
    unsafe fn EXTI1_Handler();
    unsafe fn EXTI2_Handler();
    unsafe fn EXTI3_Handler();
    unsafe fn EXTI4_Handler();
    unsafe fn EXTI9_5_Handler();
    unsafe fn FLASH_Handler();
    unsafe fn FMC_Handler();
    unsafe fn FMPI2C1_Handler();
    unsafe fn FMPI2C1_error_Handler();
    unsafe fn HDMI_CEC_Handler();
    unsafe fn I2C1_ER_Handler();
    unsafe fn I2C1_EV_Handler();
    unsafe fn I2C2_ER_Handler();
    unsafe fn I2C2_EV_Handler();
    unsafe fn I2C3_ER_Handler();
    unsafe fn I2C3_EV_Handler();
    unsafe fn OTG_FS_Handler();
    unsafe fn OTG_FS_WKUP_Handler();
    unsafe fn OTG_HS_EP1_IN_Handler();
    unsafe fn OTG_HS_EP1_OUT_Handler();
    unsafe fn OTG_HS_Handler();
    unsafe fn OTG_HS_WKUP_Handler();
    unsafe fn PVD_Handler();
    unsafe fn QuadSPI_Handler();
    unsafe fn RCC_Handler();
    unsafe fn RTC_Alarm_Handler();
    unsafe fn RTC_WKUP_Handler();
    unsafe fn SAI1_Handler();
    unsafe fn SAI2_Handler();
    unsafe fn SDIO_Handler();
    unsafe fn SPDIF_Rx_Handler();
    unsafe fn SPI1_Handler();
    unsafe fn SPI2_Handler();
    unsafe fn SPI3_Handler();
    unsafe fn SPI4_Handler();
    unsafe fn TAMP_STAMP_Handler();
    unsafe fn TIM1_BRK_TIM9_Handler();
    unsafe fn TIM1_CC_Handler();
    unsafe fn TIM1_TRG_COM_TIM11_Handler();
    unsafe fn TIM1_UP_TIM10_Handler();
    unsafe fn TIM2_Handler();
    unsafe fn TIM3_Handler();
    unsafe fn TIM4_Handler();
    unsafe fn TIM5_Handler();
    unsafe fn TIM6_DAC_Handler();
    unsafe fn TIM7_Handler();
    unsafe fn TIM8_BRK_TIM12_Handler();
    unsafe fn TIM8_CC_Handler();
    unsafe fn TIM8_TRG_COM_TIM14_Handler();
    unsafe fn TIM8_UP_TIM13_Handler();
    unsafe fn UART4_Handler();
    unsafe fn UART5_Handler();
    unsafe fn USART1_Handler();
    unsafe fn USART2_Handler();
    unsafe fn USART3_Handler();
    unsafe fn USART6_Handler();
    unsafe fn WWDG_Handler();
}

// Define Vector Table for STM32F446
#[used]
#[unsafe(link_section = ".isr_vector")]
static VECTOR_TABLE: [Option<unsafe extern "C" fn()>; 112] = [
    Some(Reset_Handler),
    Some(NMI_Handler),
    Some(HardFault_Handler),
    Some(MemManage_Handler),
    Some(BusFault_Handler),
    Some(UsageFault_Handler),
    None,
    None,
    None,
    None,
    Some(SVCall_Handler),
    Some(DebugMon_Handler),
    None,
    Some(PendSV_Handler),
    Some(SysTick_Handler),
    Some(WWDG_Handler),
    Some(PVD_Handler),
    Some(TAMP_STAMP_Handler),
    Some(RTC_WKUP_Handler),
    Some(FLASH_Handler),
    Some(RCC_Handler),
    Some(EXTI0_Handler),
    Some(EXTI1_Handler),
    Some(EXTI2_Handler),
    Some(EXTI3_Handler),
    Some(EXTI4_Handler),
    Some(DMA1_Stream0_Handler),
    Some(DMA1_Stream1_Handler),
    Some(DMA1_Stream2_Handler),
    Some(DMA1_Stream3_Handler),
    Some(DMA1_Stream4_Handler),
    Some(DMA1_Stream5_Handler),
    Some(DMA1_Stream6_Handler),
    Some(ADC_Handler),
    Some(CAN1_TX_Handler),
    Some(CAN1_RX0_Handler),
    Some(CAN1_RX1_Handler),
    Some(CAN1_SCE_Handler),
    Some(EXTI9_5_Handler),
    Some(TIM1_BRK_TIM9_Handler),
    Some(TIM1_UP_TIM10_Handler),
    Some(TIM1_TRG_COM_TIM11_Handler),
    Some(TIM1_CC_Handler),
    Some(TIM2_Handler),
    Some(TIM3_Handler),
    Some(TIM4_Handler),
    Some(I2C1_EV_Handler),
    Some(I2C1_ER_Handler),
    Some(I2C2_EV_Handler),
    Some(I2C2_ER_Handler),
    Some(SPI1_Handler),
    Some(SPI2_Handler),
    Some(USART1_Handler),
    Some(USART2_Handler),
    Some(USART3_Handler),
    Some(EXTI15_10_Handler),
    Some(RTC_Alarm_Handler),
    Some(OTG_FS_WKUP_Handler),
    Some(TIM8_BRK_TIM12_Handler),
    Some(TIM8_UP_TIM13_Handler),
    Some(TIM8_TRG_COM_TIM14_Handler),
    Some(TIM8_CC_Handler),
    Some(DMA1_Stream7_Handler),
    Some(FMC_Handler),
    Some(SDIO_Handler),
    Some(TIM5_Handler),
    Some(SPI3_Handler),
    Some(UART4_Handler),
    Some(UART5_Handler),
    Some(TIM6_DAC_Handler),
    Some(TIM7_Handler),
    Some(DMA2_Stream0_Handler),
    Some(DMA2_Stream1_Handler),
    Some(DMA2_Stream2_Handler),
    Some(DMA2_Stream3_Handler),
    Some(DMA2_Stream4_Handler),
    None,
    None,
    Some(CAN2_TX_Handler),
    Some(CAN2_RX0_Handler),
    Some(CAN2_RX1_Handler),
    Some(CAN2_SCE_Handler),
    Some(OTG_FS_Handler),
    Some(DMA2_Stream5_Handler),
    Some(DMA2_Stream6_Handler),
    Some(DMA2_Stream7_Handler),
    Some(USART6_Handler),
    Some(I2C3_EV_Handler),
    Some(I2C3_ER_Handler),
    Some(OTG_HS_EP1_OUT_Handler),
    Some(OTG_HS_EP1_IN_Handler),
    Some(OTG_HS_WKUP_Handler),
    Some(OTG_HS_Handler),
    Some(DCMI_Handler),
    None,
    None,
    Some(FPU_Handler),
    None,
    None,
    Some(SPI4_Handler),
    None,
    None,
    Some(SAI1_Handler),
    None,
    None,
    None,
    Some(SAI2_Handler),
    Some(QuadSPI_Handler),
    Some(HDMI_CEC_Handler),
    Some(SPDIF_Rx_Handler),
    Some(FMPI2C1_Handler),
    Some(FMPI2C1_error_Handler),
];

unsafe extern "C" {
    unsafe static _sidata: u32; /* Start of .data section in flash */
    unsafe static mut _sdata: u32;  /* start of .data section in RAM */
    unsafe static mut _edata: u32;  /* end of .data section in RAM */
    unsafe static mut _sbss: u32;   /* Start of .bss in RAM */
    unsafe static mut _ebss: u32;   /* End of .bss in RAM */
}

#[unsafe(no_mangle)]
extern "C" fn Default_Handler()
{ 
    // Default handler for unhandled exceptions
    loop {
        // Optionally, you can add some error handling or logging here
    }
}

#[unsafe(no_mangle)]
extern "C" fn Reset_Handler()
{
        unsafe {
        //1. Copy the .data section from FALSH to RAM
        //reference of static variable to C like raw pointer. 
        let mut src_is_flash = ptr::addr_of!(_sidata) ;
        let mut dest_is_ram: *mut u32  =    ptr::addr_of_mut!(_sdata);
        //let mut dest_is_ram: *mut u32  =    &mut _sdata as *mut u32;
        let data_end_in_ram = ptr::addr_of_mut!(_edata);

        while dest_is_ram < data_end_in_ram {
            *dest_is_ram = *src_is_flash;
            dest_is_ram = dest_is_ram.add(1);
            src_is_flash = src_is_flash.add(1);
        }

    
        //2. Zero out the .bss section in the RAM 
        let mut bss = ptr::addr_of_mut!(_sbss);
        let  bss_end = ptr::addr_of_mut!(_ebss);
        while bss < bss_end {
            *bss = 0;
            bss = bss.add(1);
        }
        
    }

    //3. call main()
    crate::main();
}


#[unsafe(no_mangle)]
extern "C" fn HardFault_Handler() { loop {} }

#[unsafe(no_mangle)]
extern "C" fn NMI_Handler() { loop {} }

