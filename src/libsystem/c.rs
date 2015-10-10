pub use imp::c as imp;

pub mod prelude {
    pub use super::imp::{
        EINVAL, EIO,
        strlen
    };
}
