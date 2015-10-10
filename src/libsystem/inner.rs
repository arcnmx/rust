pub mod prelude {
    pub use super::{AsInner, AsInnerMut, IntoInner, FromInner};
}

/// A trait for viewing representations from std types
pub trait AsInner<Inner: ?Sized> {
    fn as_inner(&self) -> &Inner;
}

/// A trait for viewing representations from std types
pub trait AsInnerMut<Inner: ?Sized> {
    fn as_inner_mut(&mut self) -> &mut Inner;
}

/// A trait for extracting representations from std types
pub trait IntoInner<Inner> {
    fn into_inner(self) -> Inner;
}

/// A trait for creating std types from internal representations
pub trait FromInner<Inner> {
    fn from_inner(inner: Inner) -> Self;
}

#[macro_export]
macro_rules! impl_inner {
    (for<$T:ident> $t:ident($t0:ident($TT:ident)): AsInner) => {
        impl<$T> $crate::inner::AsInner<$T> for $t where $t0: $crate::inner::AsInner<$T> {
            fn as_inner(&self) -> &$T { $crate::inner::AsInner::<$T>::as_inner(&self.0) }
        }
    };

    (for<$T:ident> $t:ident($t0:ident($TT:ident)): IntoInner) => {
        impl<$T> $crate::inner::IntoInner<$T> for $t where $t0: $crate::inner::IntoInner<$T> {
            fn into_inner(self) -> $T { $crate::inner::IntoInner::<$T>::into_inner(self.0) }
        }
    };

    (for<$T:ident> $t:ident($t0:ident($TT:ident)): IntoInnerForget) => {
        impl<$T> $crate::inner::IntoInner<$T> for $t where $t0: $crate::inner::IntoInner<$T> {
            fn into_inner(self) -> $T {
                let inner = $crate::inner::IntoInner::<$T>::into_inner(self.0);
                ::core::mem::forget(self);
                inner
            }
        }
    };

    (for<$T:ident> $t:ident($t0:ident($TT:ident)): FromInner) => {
        impl<$T> $crate::inner::FromInner<$T> for $t where $t0: $crate::inner::FromInner<$T> {
            fn from_inner(inner: $T) -> $t { $t($crate::inner::FromInner::<$T>::from_inner(inner)) }
        }
    };

    (1 => $t:ident($t0:ident($inner:ty)): AsInner) => {
        impl $crate::inner::AsInner<$inner> for $t {
            fn as_inner(&self) -> &$inner { $crate::inner::AsInner::<$inner>::as_inner($crate::inner::AsInner::<$t0>::as_inner(self)) }
        }
    };

    (1 => $t:ident($t0:ident($inner:ty)): IntoInner) => {
        impl $crate::inner::IntoInner<$inner> for $t {
            fn into_inner(self) -> $inner { $crate::inner::IntoInner::<$inner>::into_inner($crate::inner::IntoInner::<$t0>::into_inner(self)) }
        }
    };

    (1 => $t:ident($t0:ident($inner:ty)): FromInner) => {
        impl $crate::inner::FromInner<$inner> for $t {
            fn from_inner(inner: $inner) -> Self { $crate::inner::FromInner::<$t0>::from_inner($crate::inner::FromInner::<$inner>::from_inner(inner)) }
        }
    };

    (0 => $t:ident($inner:ty): AsInner) => {
        impl $crate::inner::AsInner<$inner> for $t {
            fn as_inner(&self) -> &$inner { &self.0 }
        }
    };

    (0 => $t:ident($inner:ty): IntoInnerForget) => {
        impl $crate::inner::IntoInner<$inner> for $t {
            fn into_inner(self) -> $inner {
                let inner = self.0;
                ::core::mem::forget(self);
                inner
            }
        }
    };

    (0 => $t:ident($inner:ty): IntoInner) => {
        impl $crate::inner::IntoInner<$inner> for $t {
            fn into_inner(self) -> $inner { self.0 }
        }
    };

    (0 => $t:ident($inner:ty): FromInner) => {
        impl $crate::inner::FromInner<$inner> for $t {
            fn from_inner(inner: $inner) -> Self { $t(inner) }
        }
    };

    ($t:ident($t0:ident($inner:ty)): $im:ident) => { impl_inner!(1 => $t($t0($inner)): $im); };
    ($t:ident($inner:ty): $im:ident) => { impl_inner!(0 => $t($inner): $im); };

    ($t:ident($t0:ident($inner:ty)): $im0:ident $(+ $im:ident)+) => {
        $(
            impl_inner!(1 =>$t($t0($inner)): $im);
        )+

        impl_inner!(1 => $t($t0($inner)): $im0);
    };

    ($t:ident($inner:ty): $im0:ident $(+ $im:ident)+) => {
        $(
            impl_inner!(0 => $t($inner): $im);
        )+

        impl_inner!(0 => $t($inner): $im0);
    };

    (for<$T:ident> $t:ident($t0:ident($TT:ident)): $im0:ident $(+ $im:ident)+) => {
        $(
            impl_inner!(for<$T> $t($t0($TT)): $im);
        )+

        impl_inner!(for<$T> $t($t0($TT)): $im0);
    };

    ($t:ident($($tt:tt)*)) => {
        impl_inner!($t($($tt)*): AsInner + IntoInner + FromInner);
    };

    (for<$T:ident> $t:ident($($tt:tt)*)) => {
        impl_inner!(for<$T> $t($($tt)*): AsInner + IntoInner + FromInner);
    };
}
