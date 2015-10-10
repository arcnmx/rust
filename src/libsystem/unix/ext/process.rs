// Copyright 2015 The Rust Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution and at
// http://rust-lang.org/COPYRIGHT.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

//! Unix-specific extensions to primitives in the `std::process` module.

#![stable(feature = "rust1", since = "1.0.0")]

use inner::prelude::*;
use os::unix::raw::{uid_t, gid_t};
use os::unix::io::{FromRawFd, RawFd, AsRawFd, IntoRawFd};
use process::prelude as sys;

/// Unix-specific extensions to the `std::process::Command` builder
#[stable(feature = "rust1", since = "1.0.0")]
pub trait CommandExt {
    /// Sets the child process's user id. This translates to a
    /// `setuid` call in the child process. Failure in the `setuid`
    /// call will cause the spawn to fail.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn uid(&mut self, id: uid_t) -> &mut Self;

    /// Similar to `uid`, but sets the group id of the child process. This has
    /// the same semantics as the `uid` field.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn gid(&mut self, id: gid_t) -> &mut Self;

    /// Create a new session (cf. `setsid(2)`) for the child process. This means
    /// that the child is the leader of a new process group. The parent process
    /// remains the child reaper of the new process.
    ///
    /// This is not enough to create a daemon process. The *init* process should
    /// be the child reaper of a daemon. This can be achieved if the parent
    /// process exit. Moreover, a daemon should not have a controlling terminal.
    /// To achieve this, a session leader (the child) must spawn another process
    /// (the daemon) in the same session.
    #[unstable(feature = "process_session_leader", reason = "recently added",
               issue = "27811")]
    fn session_leader(&mut self, on: bool) -> &mut Self;
}

#[stable(feature = "rust1", since = "1.0.0")]
impl CommandExt for sys::Command {
    fn uid(&mut self, id: uid_t) -> &mut sys::Command {
        self.as_inner_mut().uid = Some(id);
        self
    }

    fn gid(&mut self, id: gid_t) -> &mut sys::Command {
        self.as_inner_mut().gid = Some(id);
        self
    }

    fn session_leader(&mut self, on: bool) -> &mut sys::Command {
        self.as_inner_mut().session_leader = on;
        self
    }
}

/// Unix-specific extensions to `std::process::ExitStatus`
#[stable(feature = "rust1", since = "1.0.0")]
pub trait ExitStatusExt {
    /// If the process was terminated by a signal, returns that signal.
    #[stable(feature = "rust1", since = "1.0.0")]
    fn signal(&self) -> Option<i32>;
}

#[stable(feature = "rust1", since = "1.0.0")]
impl ExitStatusExt for sys::ExitStatus {
    fn signal(&self) -> Option<i32> {
        match *self.as_inner() {
            sys::ExitStatus::Signal(s) => Some(s),
            _ => None
        }
    }
}

#[stable(feature = "process_extensions", since = "1.2.0")]
impl FromRawFd for sys::Stdio {
    unsafe fn from_raw_fd(fd: RawFd) -> sys::Stdio {
        sys::Stdio::from_inner(sys::fd::FileDesc::new(fd))
    }
}
