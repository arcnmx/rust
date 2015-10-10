// Copyright 2015 The Rust Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution and at
// http://rust-lang.org/COPYRIGHT.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

//! Unix-specific extension to the primitives in the `std::ffi` module

#![stable(feature = "rust1", since = "1.0.0")]

use os_str::prelude::*;
use collections::Vec;

/// Unix-specific extensions to `OsString`.
#[stable(feature = "rust1", since = "1.0.0")]
pub trait OsStringExt {
    /// Creates an `OsString` from a byte vector.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn from_vec(vec: Vec<u8>) -> Self;

    /// Yields the underlying byte vector of this `OsString`.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn into_vec(self) -> Vec<u8>;
}

/// Unix-specific extensions to `OsStr`.
#[stable(feature = "rust1", since = "1.0.0")]
pub trait OsStrExt {
    #[stable(feature = "rust1", since = "1.0.0")]
    fn from_bytes(slice: &[u8]) -> &Self;

    /// Gets the underlying byte view of the `OsStr` slice.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn as_bytes(&self) -> &[u8];
}
