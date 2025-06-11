#![no_std]
#![no_main]
#![allow(clippy::empty_loop)]

mod startup_stm32f446;

use core::panic::PanicInfo;

static mut TEST_ARRAY: [i32; 5] = [1, 2, 3, 4, 5];

const _TEST_CONST_ARRAY: [i32; 5] = [6, 7, 8, 9, 10];

static mut _TEST_BUFFER: [u8; 10] = [0; 10];


#[unsafe(no_mangle)]
fn main() -> !
{
    let mut _test = 0;

    unsafe
    {
        for sum in TEST_ARRAY
        {
            _test += sum;
        }
    }

    loop
    {
        // Main loop of the program
        // You can add your application logic here
        
    }
}

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> !
{
    // Handle panic here, e.g., log the panic message
    loop {}
}
