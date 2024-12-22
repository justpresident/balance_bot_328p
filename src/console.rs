use core::cell::RefCell;
use avr_device::interrupt;

type Console = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
pub(crate) static CONSOLE: interrupt::Mutex<RefCell<Option<Console>>> =
    interrupt::Mutex::new(RefCell::new(None));

// Use #[macro_export] if this is moved to a separate crate:
// https://stackoverflow.com/questions/26731243/how-do-i-use-a-macro-across-module-files
#[allow(unused_macros)]
macro_rules! print {
    ($($t:tt)*) => {
        interrupt::free(
            |cs| {
                if let Some(console) = console::CONSOLE.borrow(cs).borrow_mut().as_mut() {
                    let _ = ufmt::uwrite!(console, $($t)*);
                }
            },
        )
    };
}
#[allow(unused_imports)]
pub(crate) use print;

macro_rules! println {
    ($($t:tt)*) => {
        interrupt::free(
            |cs| {
                if let Some(console) = console::CONSOLE.borrow(cs).borrow_mut().as_mut() {
                    let _ = ufmt::uwriteln!(console, $($t)*);
                }
            },
        )
    };
}
pub(crate) use println;

pub fn set_console(console: Console) {
    interrupt::free(|cs| {
        *CONSOLE.borrow(cs).borrow_mut() = Some(console);
    })
}
