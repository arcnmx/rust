pub mod backtrace {
    use backtrace as sys;
    use error::prelude::*;

    pub struct Backtrace(());

    impl Backtrace {
        pub const fn new() -> Self { Backtrace(()) }
    }

    impl sys::Backtrace for Backtrace {
        fn write<O: sys::BacktraceOutput>(&mut self, w: &mut O) -> Result<()> {
            rtabort!("backtrace disabled")
        }

        fn log_enabled() -> bool {
            false
        }
    }
}
