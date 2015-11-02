#![allow(improper_ctypes)]

pub type Handle = usize;
pub type NZHandle = ::core::nonzero::NonZero<usize>;

pub mod thread_local {
    use ptr::null_mut;
    use cell::UnsafeCell;
    use marker::Sync;

    pub struct StaticOsKey(UnsafeCell<*mut u8>, Option<unsafe extern fn(*mut u8)>);
    pub struct OsKey(UnsafeCell<*mut u8>, Option<unsafe extern fn(*mut u8)>);
    pub struct Key<T>(UnsafeCell<Option<T>>);
    unsafe impl<T> Sync for Key<T> { }
    unsafe impl<T> Send for Key<T> { }
    unsafe impl Sync for OsKey { }
    unsafe impl Send for OsKey { }
    unsafe impl Sync for StaticOsKey { }
    unsafe impl Send for StaticOsKey { }

    impl<T> Key<T> {
        pub unsafe fn get(&'static self) -> Option<&'static UnsafeCell<Option<T>>> {
            Some(&self.0)
        }
    }

    impl<T> Key<T> {
        pub const fn new() -> Self { Key(UnsafeCell::new(None)) }
    }

    impl StaticOsKey {
        pub const fn new(dtor: Option<unsafe extern fn(*mut u8)>) -> Self { StaticOsKey(UnsafeCell::new(null_mut()), dtor) }
    }

    impl StaticOsKey {
        pub unsafe fn get(&self) -> *mut u8 { *self.0.get() }
        pub unsafe fn set(&self, val: *mut u8) { *self.0.get() = val }
        pub unsafe fn destroy(&self) {
            if let Some(dtor) = self.1 {
                dtor(self.get())
            }
        }
    }

    impl OsKey {
        pub fn new(dtor: Option<unsafe extern fn(*mut u8)>) -> Self { OsKey(UnsafeCell::new(null_mut()), dtor) }
        pub fn get(&self) -> *mut u8 { unsafe { *self.0.get() } }
        pub fn set(&self, val: *mut u8) { unsafe { *self.0.get() = val } }
    }
}

pub mod error {
    use core::nonzero::NonZero;
	use borrow::Cow;
    use fmt;
	use io;

    pub use sys::common::error::{Result, expect_last_result, expect_last_error};

    pub type ErrorCode = NonZero<i32>;

    extern {
        fn __rust_bind_error_last() -> Option<ErrorCode>;
        fn __rust_bind_error_string(code: i32, buffer: &mut [u8]) -> Option<usize>;
        fn __rust_bind_error_kind(code: i32) -> io::ErrorKind;
    }

    const BUFSIZE: usize = 0x80;

    pub struct Error(ErrorCode);
    pub struct ErrorString([u8; BUFSIZE], usize);
    
    impl Error {
        pub fn from_code(code: i32) -> Self {
            debug_assert!(code != 0);
            unsafe { Error(NonZero::new(code)) }
        }

        pub fn last_error() -> Option<Self> {
            let code = unsafe {
                __rust_bind_error_last()
            };

            code.map(Error)
        }

        pub fn default() -> Self {
            Self::from_code(-1)
        }

        pub fn code(&self) -> i32 { *self.0 }
        pub fn description(&self) -> ErrorString {
            let mut s = [0u8; BUFSIZE];
            let len = unsafe {
                __rust_bind_error_string(self.code(), &mut s)
            };
            ErrorString(s, len.unwrap_or(0))
        }

		pub fn kind(&self) -> io::ErrorKind {
			unsafe { __rust_bind_error_kind(self.code()) }
		}
    }

    impl ErrorString {
        pub fn as_bytes(&self) -> &[u8] {
            &self.0[..self.1]
        }

		pub fn to_string_lossy(&self) -> Cow<str> {
			use string::String;

			String::from_utf8_lossy(self.as_bytes())
		}
    }

    impl From<fmt::Error> for Error {
        fn from(_: fmt::Error) -> Self {
			unimplemented!()
        }
    }
}

pub mod time {
    use time::Duration;
	use sys::error::Result;

    extern {
        fn __rust_bind_time_steady_now() -> Result<u64>;
    }

    const NANOS_PER_SEC: u64 = 1_000_000_000;

    pub struct SteadyTime(u64);

    impl SteadyTime {
        pub fn now() -> Result<Self> {
            unsafe {
                __rust_bind_time_steady_now().map(SteadyTime)
            }
        }

        pub fn delta(&self, rhs: &Self) -> Duration {
            let diff = self.0 - rhs.0;
            let secs = diff / NANOS_PER_SEC;
            let nanos = diff % NANOS_PER_SEC;
            Duration::new(secs, nanos as u32)
        }
    }
}

pub mod sync {
    use time::Duration;
    use marker;
    use cell::UnsafeCell;
    use super::Handle;

    extern {
        fn __rust_bind_sync_mutex_lock(handle: *mut Handle);
        fn __rust_bind_sync_mutex_unlock(handle: *mut Handle);
        fn __rust_bind_sync_mutex_try_lock(handle: *mut Handle) -> bool;
        fn __rust_bind_sync_mutex_destroy(handle: *mut Handle);

        fn __rust_bind_sync_remutex_init(handle: *mut Handle);
        fn __rust_bind_sync_remutex_lock(handle: *mut Handle);
        fn __rust_bind_sync_remutex_unlock(handle: *mut Handle);
        fn __rust_bind_sync_remutex_try_lock(handle: *mut Handle) -> bool;
        fn __rust_bind_sync_remutex_destroy(handle: *mut Handle);

        fn __rust_bind_sync_rwlock_read(handle: *mut Handle);
        fn __rust_bind_sync_rwlock_try_read(handle: *mut Handle) -> bool;
        fn __rust_bind_sync_rwlock_write(handle: *mut Handle);
        fn __rust_bind_sync_rwlock_try_write(handle: *mut Handle) -> bool;
        fn __rust_bind_sync_rwlock_read_unlock(handle: *mut Handle);
        fn __rust_bind_sync_rwlock_write_unlock(handle: *mut Handle);
        fn __rust_bind_sync_rwlock_destroy(handle: *mut Handle);

        fn __rust_bind_sync_condvar_notify_one(handle: *mut Handle);
        fn __rust_bind_sync_condvar_notify_all(handle: *mut Handle);
        fn __rust_bind_sync_condvar_wait(handle: *mut Handle, mutex: *mut Handle);
        fn __rust_bind_sync_condvar_wait_timeout(handle: *mut Handle, mutex: *mut Handle, dur: Duration) -> bool;
        fn __rust_bind_sync_condvar_destroy(handle: *mut Handle);

        fn __rust_bind_sync_once(handle: *mut Handle, f: unsafe extern fn(usize), data: usize);
    }

    pub struct Mutex(UnsafeCell<Handle>);
    pub struct ReentrantMutex(UnsafeCell<Handle>);
    pub struct RwLock(UnsafeCell<Handle>);
    pub struct Condvar(UnsafeCell<Handle>);

    impl ReentrantMutex {
		pub unsafe fn lock(&self) { __rust_bind_sync_remutex_lock(self.0.get()) }
        pub unsafe fn unlock(&self) { __rust_bind_sync_remutex_unlock(self.0.get()) }
        pub unsafe fn try_lock(&self) -> bool { __rust_bind_sync_remutex_try_lock(self.0.get()) }
        pub unsafe fn destroy(&self) { __rust_bind_sync_remutex_destroy(self.0.get()) }
    }

    impl Mutex {
        pub unsafe fn lock(&self) { __rust_bind_sync_mutex_lock(self.0.get()) }
        pub unsafe fn unlock(&self) { __rust_bind_sync_mutex_unlock(self.0.get()) }
        pub unsafe fn try_lock(&self) -> bool { __rust_bind_sync_mutex_try_lock(self.0.get()) }
        pub unsafe fn destroy(&self) { __rust_bind_sync_mutex_destroy(self.0.get()) }
    }

    impl Mutex {
        pub const fn new() -> Mutex { Mutex(UnsafeCell::new(0)) }
    }

    impl ReentrantMutex {
        pub const fn uninitialized() -> ReentrantMutex { ReentrantMutex(UnsafeCell::new(0)) }
    }

    impl ReentrantMutex {
        pub unsafe fn init(&mut self) { __rust_bind_sync_remutex_init(self.0.get()) }
    }

    impl RwLock {
        pub const fn new() -> RwLock { RwLock(UnsafeCell::new(0)) }
    }

    impl RwLock {
        pub unsafe fn read(&self) { __rust_bind_sync_rwlock_read(self.0.get()) }
        pub unsafe fn try_read(&self) -> bool { __rust_bind_sync_rwlock_try_read(self.0.get()) }
        pub unsafe fn write(&self) { __rust_bind_sync_rwlock_write(self.0.get()) }
        pub unsafe fn try_write(&self) -> bool { __rust_bind_sync_rwlock_try_write(self.0.get()) }
        pub unsafe fn read_unlock(&self) { __rust_bind_sync_rwlock_read_unlock(self.0.get()) }
        pub unsafe fn write_unlock(&self) { __rust_bind_sync_rwlock_write_unlock(self.0.get()) }
        pub unsafe fn destroy(&self) { __rust_bind_sync_rwlock_destroy(self.0.get()) }
    }

    impl Condvar {
        pub const fn new() -> Condvar { Condvar(UnsafeCell::new(0)) }
    }

    impl Condvar {
        pub unsafe fn notify_one(&self) { __rust_bind_sync_condvar_notify_one(self.0.get()) }
        pub unsafe fn notify_all(&self) { __rust_bind_sync_condvar_notify_all(self.0.get()) }
        pub unsafe fn wait(&self, mutex: &Mutex) { __rust_bind_sync_condvar_wait(self.0.get(), mutex.0.get()) }
        pub unsafe fn wait_timeout(&self, mutex: &Mutex, dur: Duration) -> bool { __rust_bind_sync_condvar_wait_timeout(self.0.get(), mutex.0.get(), dur) }
        pub unsafe fn destroy(&self) { __rust_bind_sync_condvar_destroy(self.0.get()) }
    }

    unsafe impl marker::Sync for Mutex { }
    unsafe impl marker::Send for Mutex { }
    unsafe impl marker::Sync for ReentrantMutex { }
    unsafe impl marker::Send for ReentrantMutex { }
    unsafe impl marker::Sync for RwLock { }
    unsafe impl marker::Send for RwLock { }
    unsafe impl marker::Sync for Condvar { }
    unsafe impl marker::Send for Condvar { }
}

pub mod stdio {
	use sys::error::Result;
	use io;

	pub use sys::common::stdio::dumb_print;

    extern {
        fn __rust_bind_stdio_stdin(buf: &mut [u8]) -> io::Result<usize>;
        fn __rust_bind_stdio_stdout(buf: &[u8]) -> io::Result<usize>;
        fn __rust_bind_stdio_stdout_flush() -> io::Result<()>;
        fn __rust_bind_stdio_stderr(buf: &[u8]) -> io::Result<usize>;
        fn __rust_bind_stdio_stderr_flush() -> io::Result<()>;
        fn __rust_bind_stdio_is_ebadf(e: &io::Error) -> bool;
    }

    pub struct Stdin(());
    pub struct Stdout(());
    pub struct Stderr(());

    impl io::Read for Stdin {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            unsafe { __rust_bind_stdio_stdin(buf) }
        }
    }

    impl io::Write for Stdout {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            unsafe { __rust_bind_stdio_stdout(buf) }
        }

		fn flush(&mut self) -> io::Result<()> {
            unsafe { __rust_bind_stdio_stdout_flush() }
		}
    }

    impl io::Write for Stderr {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            unsafe { __rust_bind_stdio_stderr(buf) }
        }

		fn flush(&mut self) -> io::Result<()> {
            unsafe { __rust_bind_stdio_stderr_flush() }
		}
    }

	pub fn stdin() -> Result<Stdin> { Ok(Stdin(())) }
	pub fn stdout() -> Result<Stdout> { Ok(Stdout(())) }
	pub fn stderr() -> Result<Stderr> { Ok(Stderr(())) }

	pub fn is_ebadf(e: &io::Error) -> bool {
		unsafe { __rust_bind_stdio_is_ebadf(e) }
	}
}

pub mod stack_overflow {
    use super::NZHandle as Handle;

    extern {
        fn __rust_bind_stack_overflow_handler() -> Option<Handle>;
        fn __rust_bind_stack_overflow_handler_drop(handle: Handle);
        fn __rust_bind_stack_overflow_init();
        fn __rust_bind_stack_overflow_cleanup();
    }

    pub struct Handler(Option<Handle>);

    impl Handler {
        pub unsafe fn new() -> Self {
            Handler(__rust_bind_stack_overflow_handler())
        }
    }

    impl Drop for Handler {
        fn drop(&mut self) {
            if let Some(ref handle) = self.0 {
                unsafe { __rust_bind_stack_overflow_handler_drop(*handle) }
            }
        }
    }

    pub unsafe fn init() {
        __rust_bind_stack_overflow_init();
    }

    pub unsafe fn cleanup() {
        __rust_bind_stack_overflow_cleanup();
    }
}

pub mod rand {
    use rand;
    use mem;
    use sys::error::Result;
    use super::NZHandle as Handle;

    extern {
        fn __rust_bind_rand_new() -> Result<Handle>;
        fn __rust_bind_rand_fill(handle: Handle, buf: &mut [u8]) -> Result<()>;
        fn __rust_bind_rand_drop(handle: Handle) -> Result<()>;
    }

    pub struct OsRng(Handle);

    impl OsRng {
        pub fn new() -> Result<Self> { unsafe { __rust_bind_rand_new().map(OsRng) } }
    }

    impl rand::Rng for OsRng {
        fn next_u32(&mut self) -> u32 {
            let mut data = [0u8; 4];
            self.fill_bytes(&mut data);
            unsafe { mem::transmute(data) }
        }

        fn next_u64(&mut self) -> u64 {
            let mut data = [0u8; 8];
            self.fill_bytes(&mut data);
            unsafe { mem::transmute(data) }
        }

        fn fill_bytes(&mut self, v: &mut [u8]) {
            let res = unsafe { __rust_bind_rand_fill(self.0, v) };
            res.expect("Rng failed to produce bytes")
        }
    }

    impl Drop for OsRng {
        fn drop(&mut self) {
            let _ = unsafe { __rust_bind_rand_drop(self.0) };
        }
    }
}

pub mod thread {
    use sys::error::Result;
    use time::Duration;
    use super::NZHandle as Handle;

    extern {
        fn __rust_bind_thread_new(stack_size: usize, f: unsafe extern fn(usize) -> usize, data: usize) -> Result<Handle>;
        fn __rust_bind_thread_join(handle: Handle) -> Result<()>;
        fn __rust_bind_thread_drop(handle: Handle);
        fn __rust_bind_thread_set_name(name: &str) -> Result<()>;
        fn __rust_bind_thread_yield();
        fn __rust_bind_thread_sleep(dur: Duration) -> Result<()>;
    }

    pub struct Thread(Handle);

	pub unsafe fn new(stack: usize, f: unsafe extern fn(usize) -> usize, data: usize) -> Result<Thread> {
		__rust_bind_thread_new(stack, f, data).map(Thread)
	}

    impl Thread {
        pub fn join(self) -> Result<()> { unsafe { __rust_bind_thread_join(self.0) } }
	}

	pub fn set_name(name: &str) -> Result<()> { unsafe { __rust_bind_thread_set_name(name) } }
	pub fn yield_() { unsafe { __rust_bind_thread_yield() } }
	pub fn sleep(dur: Duration) -> Result<()> { unsafe { __rust_bind_thread_sleep(dur) } }

    impl Drop for Thread {
        fn drop(&mut self) {
            let _ = unsafe { __rust_bind_thread_drop(self.0) };
        }
    }
}

pub mod unwind {
    use any::Any;
    use boxed::Box;
    use fmt;

    extern {
        fn __rust_bind_unwind_begin_unwind_fmt(msg: fmt::Arguments, file_line: &(&'static str, u32)) -> !;
        fn __rust_bind_unwind_begin_unwind(msg: &Any, file_line: &(&'static str, u32)) -> !;
        fn __rust_bind_unwind_panic_inc() -> usize;
        fn __rust_bind_unwind_is_panicking() -> bool;
        fn __rust_bind_unwind_try(f: unsafe extern fn(usize), data: usize) -> Result<(), Box<Any + Send>>;
    }

	pub fn begin_unwind_fmt(msg: fmt::Arguments, file_line: &(&'static str, u32)) -> ! {
		unsafe { __rust_bind_unwind_begin_unwind_fmt(msg, file_line) }
	}

	pub fn begin_unwind<M: Any + Send>(msg: M, file_line: &(&'static str, u32)) -> ! {
		unsafe { __rust_bind_unwind_begin_unwind(&msg, file_line) }
	}

	pub fn panic_inc() -> usize {
		unsafe { __rust_bind_unwind_panic_inc() }
	}

	pub fn is_panicking() -> bool {
		unsafe { __rust_bind_unwind_is_panicking() }
	}

	pub unsafe fn try<F: FnOnce()>(f: F) -> Result<(), Box<Any + Send>> {
		let mut f = Some(f);
		unsafe extern fn __call<F: FnOnce()>(f: usize) {
			let f = f as *mut Option<F>;
			(*f).take().expect("f must only be called once")()
		}
		__rust_bind_unwind_try(__call::<F>, &mut f as *mut _ as usize)
	}
}

pub mod backtrace {
    use sys::error::Result;
    use io;

    extern {
        fn __rust_bind_backtrace_write(w: &mut io::Write) -> Result<()>;
        fn __rust_bind_backtrace_log_enabled() -> bool;
    }

    pub struct Backtrace(());

    impl Backtrace {
        pub const fn new() -> Self { Backtrace(()) }
    }

    impl Backtrace {
        pub fn write(&mut self, w: &mut io::Write) -> Result<()> {
            unsafe { __rust_bind_backtrace_write(w) }
        }
    }
}

pub mod net {
	use net::Shutdown;
    use sys::error::Result;
    use sys::inner::*;
    use io;
    use iter;
    use mem;
    use time::Duration;
    use super::NZHandle as Handle;

	pub use sys::common::net::{
		IpAddr, SocketAddr,
	};

	pub type TcpStream = Socket;
	pub type TcpListener = Socket;
	pub type UdpSocket = Socket;

    const BUFSIZE: usize = 0x400;

    pub struct LookupHost(());
    pub struct LookupAddr([u8; BUFSIZE], usize);

    #[derive(Copy, Clone, PartialOrd, Ord, PartialEq, Eq, Hash)]
    pub struct IpAddrV4([u8; 4]);

    #[derive(Copy, Clone, PartialOrd, Ord, PartialEq, Eq, Hash)]
    pub struct IpAddrV6([u16; 8]);

    #[derive(Copy, Clone, PartialEq, Eq, Hash)]
    pub struct SocketAddrV4 {
        addr: IpAddrV4,
        port: u16,
    }

    #[derive(Copy, Clone, PartialEq, Eq, Hash)]
    pub struct SocketAddrV6 {
        addr: IpAddrV6,
        port: u16,
        flowinfo: u32,
        scope_id: u32,
    }

    #[derive(Debug)]
    pub struct Socket(Handle);

    impl iter::Iterator for LookupHost {
        type Item = Result<SocketAddr>;

        fn next(&mut self) -> Option<Self::Item> { None }
    }

    impl LookupAddr {
        pub fn as_bytes(&self) -> &[u8] {
            &self.0[..self.1]
        }
    }

    impl IpAddrV4 {
        pub fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
            IpAddrV4([a, b, c, d])
        }

        pub fn octets(&self) -> [u8; 4] { self.0 }
    }

    impl IpAddrV6 {
        pub fn new(a: u16, b: u16, c: u16, d: u16, e: u16, f: u16, g: u16, h: u16) -> Self {
            IpAddrV6([a, b, c, d, e, f, g, h])
        }

        pub fn segments(&self) -> [u16; 8] { self.0 }
    }

    impl SocketAddrV4 {
        pub fn new(ip: IpAddrV4, port: u16) -> Self {
            SocketAddrV4 {
                addr: ip,
                port: port,
            }
        }

        pub fn addr(&self) -> &IpAddrV4 { &self.addr }
        pub fn port(&self) -> u16 { self.port }
    }

    impl SocketAddrV6 {
        pub fn new(ip: IpAddrV6, port: u16, flowinfo: u32, scope_id: u32) -> Self {
            SocketAddrV6 {
                addr: ip,
                port: port,
                flowinfo: flowinfo,
                scope_id: scope_id,
            }
        }

        pub fn addr(&self) -> &IpAddrV6 { &self.addr }
        pub fn port(&self) -> u16 { self.port }
        pub fn flowinfo(&self) -> u32 { self.flowinfo }
        pub fn scope_id(&self) -> u32 { self.scope_id }
    }

    extern {
        //fn __rust_bind_net_lookup_host(host: &str) -> Result<LookupHost>;
        fn __rust_bind_net_lookup_addr(addr: &IpAddr, buf: &mut [u8]) -> Result<usize>;

        fn __rust_bind_net_tcp_connect(addr: &SocketAddr) -> Result<Handle>;
        fn __rust_bind_net_tcp_bind(addr: &SocketAddr) -> Result<Handle>;
        fn __rust_bind_net_udp_bind(addr: &SocketAddr) -> Result<Handle>;

        fn __rust_bind_net_socket_read(handle: Handle, buf: &mut [u8]) -> Result<usize>;
        fn __rust_bind_net_socket_write(handle: Handle, buf: &[u8]) -> Result<usize>;
        fn __rust_bind_net_socket_flush(handle: Handle) -> Result<()>;
        fn __rust_bind_net_socket_peer_addr(handle: Handle) -> Result<SocketAddr>;
        fn __rust_bind_net_socket_shutdown(handle: Handle, how: Shutdown) -> Result<()>;

        fn __rust_bind_net_socket_accept(handle: Handle) -> Result<(Handle, SocketAddr)>;
        fn __rust_bind_net_socket_recv_from(handle: Handle, buf: &mut [u8]) -> Result<(usize, SocketAddr)>;
        fn __rust_bind_net_socket_send_to(handle: Handle, buf: &[u8], dst: &SocketAddr) -> Result<usize>;

        fn __rust_bind_net_socket_local_addr(handle: Handle) -> Result<SocketAddr>;
        fn __rust_bind_net_socket_set_read_timeout(handle: Handle, dur: Option<Duration>) -> Result<()>;
        fn __rust_bind_net_socket_set_write_timeout(handle: Handle, dur: Option<Duration>) -> Result<()>;
        fn __rust_bind_net_socket_get_read_timeout(handle: Handle) -> Result<Option<Duration>>;
        fn __rust_bind_net_socket_get_write_timeout(handle: Handle) -> Result<Option<Duration>>;
        fn __rust_bind_net_socket_duplicate(handle: Handle) -> Result<Handle>;
        fn __rust_bind_net_socket_drop(handle: Handle) -> Result<()>;
    }

	pub fn lookup_host(host: &str) -> Result<LookupHost> { panic!() }

	pub fn lookup_addr(addr: &IpAddr) -> Result<LookupAddr> {
		let mut s = [0u8; BUFSIZE];
		let len = unsafe {
			__rust_bind_net_lookup_addr(addr, &mut s)
		};
		len.map(|len| LookupAddr(s, len))
	}

	pub fn connect_tcp(addr: &SocketAddr) -> Result<TcpStream> {
		unsafe { __rust_bind_net_tcp_connect(addr).map(Socket) }
	}

	pub fn bind_tcp(addr: &SocketAddr) -> Result<TcpListener> {
		unsafe { __rust_bind_net_tcp_bind(addr).map(Socket) }
	}

	pub fn bind_udp(addr: &SocketAddr) -> Result<UdpSocket> {
		unsafe { __rust_bind_net_udp_bind(addr).map(Socket) }
    }

    impl Socket {
        pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
            unsafe { __rust_bind_net_socket_read(self.0, buf) }
        }

        pub fn write(&self, buf: &[u8]) -> Result<usize> {
            unsafe { __rust_bind_net_socket_write(self.0, buf) }
        }

        pub fn flush(&self) -> Result<()> {
            unsafe { __rust_bind_net_socket_flush(self.0) }
        }

        pub fn peer_addr(&self) -> Result<SocketAddr> {
            unsafe { __rust_bind_net_socket_peer_addr(self.0) }
        }

        pub fn shutdown(&self, how: Shutdown) -> Result<()> {
            unsafe { __rust_bind_net_socket_shutdown(self.0, how) }
        }

        pub fn accept(&self) -> Result<(Socket, SocketAddr)> {
            unsafe { __rust_bind_net_socket_accept(self.0).map(|(s, a)| (Socket(s), a)) }
        }

        pub fn recv_from(&self, buf: &mut [u8]) -> Result<(usize, SocketAddr)> {
            unsafe { __rust_bind_net_socket_recv_from(self.0, buf) }
        }

        pub fn send_to(&self, buf: &[u8], dst: &SocketAddr) -> Result<usize> {
            unsafe { __rust_bind_net_socket_send_to(self.0, buf, dst) }
        }

        pub fn socket_addr(&self) -> Result<SocketAddr> {
            unsafe { __rust_bind_net_socket_local_addr(self.0) }
        }

        pub fn set_read_timeout(&self, dur: Option<Duration>) -> Result<()> {
            unsafe { __rust_bind_net_socket_set_read_timeout(self.0, dur) }
        }

        pub fn set_write_timeout(&self, dur: Option<Duration>) -> Result<()> {
            unsafe { __rust_bind_net_socket_set_write_timeout(self.0, dur) }
        }

        pub fn read_timeout(&self) -> Result<Option<Duration>> {
            unsafe { __rust_bind_net_socket_get_read_timeout(self.0) }
        }

        pub fn write_timeout(&self) -> Result<Option<Duration>> {
            unsafe { __rust_bind_net_socket_get_write_timeout(self.0) }
        }

        pub fn duplicate(&self) -> Result<Self> {
            unsafe { __rust_bind_net_socket_duplicate(self.0).map(Socket) }
        }
    }

    impl Drop for Socket {
        fn drop(&mut self) {
            let _ = unsafe { __rust_bind_net_socket_drop(self.0) };
        }
    }

    impl_inner!(Socket(Handle): FromInner + AsInner + IntoInnerForget);

	impl AsInner<Socket> for Socket {
		fn as_inner(&self) -> &Socket {
			self
		}
	}
}

pub mod dynamic_lib {
    use ffi::OsStr;
    use sys::error::{self, Result};
    use super::NZHandle as Handle;

	pub type Error = error::Error;

    extern {
        fn __rust_bind_dynamic_lib_open(filename: Option<&OsStr>) -> Result<Handle>;
        fn __rust_bind_dynamic_lib_symbol(handle: Handle, name: &str) -> Result<*mut u8>;
        fn __rust_bind_dynamic_lib_close(handle: Handle) -> Result<()>;
    }

    pub struct DynamicLibrary(Handle);

	pub fn open(filename: Option<&OsStr>) -> Result<DynamicLibrary> {
		unsafe { __rust_bind_dynamic_lib_open(filename).map(DynamicLibrary) }
	}

    impl DynamicLibrary {
        pub fn symbol(&self, symbol: &str) -> Result<*mut u8> {
            unsafe { __rust_bind_dynamic_lib_symbol(self.0, symbol) }
        }

        pub fn close(&self) -> Result<()> {
            unsafe { __rust_bind_dynamic_lib_close(self.0) }
        }
	}

	pub const ENVVAR: &'static str = "RUST_BIND_LD";
	pub const SEPARATOR: &'static str = ":";
}

pub mod env {
    use ffi::{OsStr, OsString};
    use sys::error::Result;
	use borrow::Cow;
    use result;
    use iter;

	pub use sys::common::env::{JoinPathsError, ARCH};

    extern {
        fn __rust_bind_env_getcwd() -> Result<OsString>;
        fn __rust_bind_env_chdir(p: &OsStr) -> Result<()>;
        fn __rust_bind_env_getenv(k: &OsStr) -> Result<Option<OsString>>;
        fn __rust_bind_env_setenv(k: &OsStr, v: &OsStr) -> Result<()>;
        fn __rust_bind_env_unsetenv(k: &OsStr) -> Result<()>;
        fn __rust_bind_env_home_dir() -> Result<OsString>;
        fn __rust_bind_env_temp_dir() -> Result<OsString>;
        fn __rust_bind_env_current_exe() -> Result<OsString>;
    }

    pub type SplitPaths<'a> = iter::Empty<Cow<'a, OsStr>>;

	pub type Args = iter::Empty<OsString>;
	pub type Vars = iter::Empty<(OsString, OsString)>;

	pub fn getcwd() -> Result<OsString> {
		unsafe { __rust_bind_env_getcwd() }
	}

	pub fn chdir(p: &OsStr) -> Result<()> {
		unsafe { __rust_bind_env_chdir(p) }
	}

	pub fn getenv(k: &OsStr) -> Result<Option<OsString>> {
		unsafe { __rust_bind_env_getenv(k) }
	}

	pub fn setenv(k: &OsStr, v: &OsStr) -> Result<()> {
		unsafe { __rust_bind_env_setenv(k, v) }
	}

	pub fn unsetenv(k: &OsStr) -> Result<()> {
		unsafe { __rust_bind_env_unsetenv(k) }
	}

	pub fn home_dir() -> Result<OsString> {
		unsafe { __rust_bind_env_home_dir() }
	}

	pub fn temp_dir() -> Result<OsString> {
		unsafe { __rust_bind_env_temp_dir() }
	}

	pub fn current_exe() -> Result<OsString> {
		unsafe { __rust_bind_env_current_exe() }
	}

	pub fn vars() -> Result<Vars> { panic!() }
	pub fn args() -> Result<Args> { panic!() }

	pub fn join_paths<I: Iterator<Item=T>, T: AsRef<OsStr>>(paths: I) -> result::Result<OsString, JoinPathsError> { panic!() }
	pub fn join_paths_error() -> &'static str { "" }

	pub const FAMILY: &'static str = "";
	pub const OS: &'static str = "";
	pub const DLL_PREFIX: &'static str = "";
	pub const DLL_SUFFIX: &'static str = "";
	pub const DLL_EXTENSION: &'static str = "";
	pub const EXE_SUFFIX: &'static str = "";
	pub const EXE_EXTENSION: &'static str = "";

    pub fn split_paths(unparsed: &OsStr) -> SplitPaths { panic!() }
}

pub mod process {
    use io;
    use sys::error::Result;
    use iter;
    use fmt;
    use ffi::OsStr;
    use super::NZHandle as Handle;

	pub use sys::common::process::Stdio;

    extern {
        fn __rust_bind_process_spawn(cmd: &Command, stdin: &Stdio, stdout: &Stdio, stderr: &Stdio) -> Result<(Handle, Option<AnonPipe>, Option<AnonPipe>, Option<AnonPipe>)>;
        fn __rust_bind_process_exit(code: i32) -> !;
        fn __rust_bind_process_kill(handle: Handle) -> Result<()>;
        fn __rust_bind_process_id(handle: Handle) -> Result<u32>;
        fn __rust_bind_process_wait(handle: Handle) -> Result<ExitStatus>;
        fn __rust_bind_process_try_wait(handle: Handle) -> Option<ExitStatus>;
        fn __rust_bind_process_pipe_read(handle: Handle, buf: &mut [u8]) -> io::Result<usize>;
        fn __rust_bind_process_pipe_write(handle: Handle, buf: &[u8]) -> io::Result<usize>;
        fn __rust_bind_process_pipe_flush(handle: Handle) -> io::Result<()>;
        fn __rust_bind_process_exit_status_display(status: &ExitStatus, f: &mut fmt::Formatter) -> fmt::Result;
    }

    pub struct Process {
        handle: Handle,
        stdin: Option<AnonPipe>,
        stdout: Option<AnonPipe>,
        stderr: Option<AnonPipe>,
    }
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub struct ExitStatus(Option<i32>, Option<Handle>);
    pub struct AnonPipe(Handle);
    #[derive(Clone, Debug)]
    pub struct Command(());

    pub type RawFd = Handle;
	pub type PipeRead = AnonPipe;
	pub type PipeWrite = AnonPipe;

    impl io::Read for AnonPipe {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            unsafe { __rust_bind_process_pipe_read(self.0, buf) }
        }
    }

    impl io::Write for AnonPipe {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            unsafe { __rust_bind_process_pipe_write(self.0, buf) }
        }

        fn flush(&mut self) -> io::Result<()> {
            unsafe { __rust_bind_process_pipe_flush(self.0) }
		}
    }

    impl Command {
        pub fn new(program: &OsStr) -> Result<Self> { panic!() }

        pub fn arg(&mut self, arg: &OsStr) { panic!() }
        pub fn args<'a, I: iter::Iterator<Item = &'a OsStr>>(&mut self, args: I) { panic!() }
        pub fn env(&mut self, key: &OsStr, val: &OsStr) { panic!() }
        pub fn env_remove(&mut self, key: &OsStr) { panic!() }
        pub fn env_clear(&mut self) { panic!() }
        pub fn cwd(&mut self, dir: &OsStr) { panic!() }
    }

    impl ExitStatus {
        pub fn success(&self) -> bool { self.0 == Some(0) }
        pub fn code(&self) -> Option<i32> { self.0 }
    }

    impl fmt::Display for ExitStatus {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            unsafe {
                __rust_bind_process_exit_status_display(self, f)
            }
        }
    }

	pub fn spawn(cfg: &Command, stdin: &Stdio, stdout: &Stdio, stderr: &Stdio) -> Result<Process> {
		unsafe { __rust_bind_process_spawn(cfg, stdin, stdout, stderr)
			.map(|(h, sin, sout, serr)| Process {
				handle: h,
				stdin: sin,
				stdout: sout,
				stderr: serr,
			})
		}
	}

	pub fn exit(code: i32) -> ! {
		unsafe { __rust_bind_process_exit(code) }
	}

    impl Process {
        pub unsafe fn kill(&self) -> Result<()> {
            __rust_bind_process_kill(self.handle)
        }

        pub fn id(&self) -> Result<u32> {
            unsafe { __rust_bind_process_id(self.handle) }
        }

        pub fn wait(&self) -> Result<ExitStatus> {
            unsafe { __rust_bind_process_wait(self.handle) }
        }

        pub fn try_wait(&self) -> Option<ExitStatus> {
            unsafe { __rust_bind_process_try_wait(self.handle) }
        }

        pub fn stdin(&mut self) -> &mut Option<PipeWrite> { &mut self.stdin }
        pub fn stdout(&mut self) -> &mut Option<PipeRead> { &mut self.stdout }
        pub fn stderr(&mut self) -> &mut Option<PipeRead> { &mut self.stderr }
    }
}

pub mod fs {
    use sys::inner::*;
    use sys::error::Result;
	use borrow::Cow;
    use ffi::CStr;
    use io;
    use mem;
    use ffi::{OsStr, OsString};
    use super::NZHandle as Handle;

    extern {
        fn __rust_bind_fs_unlink(p: &OsStr) -> Result<()>;
        fn __rust_bind_fs_stat(p: &OsStr) -> Result<FileAttr>;
        fn __rust_bind_fs_lstat(p: &OsStr) -> Result<FileAttr>;
        fn __rust_bind_fs_rename(from: &OsStr, to: &OsStr) -> Result<()>;
        fn __rust_bind_fs_copy(from: &OsStr, to: &OsStr) -> Result<u64>;
        fn __rust_bind_fs_link(src: &OsStr, dst: &OsStr) -> Result<()>;
        fn __rust_bind_fs_symlink(src: &OsStr, dst: &OsStr) -> Result<()>;
        fn __rust_bind_fs_readlink(p: &OsStr) -> Result<OsString>;
        fn __rust_bind_fs_canonicalize(p: &OsStr) -> Result<OsString>;
        fn __rust_bind_fs_rmdir(p: &OsStr) -> Result<()>;
        fn __rust_bind_fs_set_perm(p: &OsStr, perm: FilePermissions) -> Result<()>;

        fn __rust_bind_fs_file_open(p: &OsStr, opt: &OpenOptions) -> Result<Handle>;
        fn __rust_bind_fs_file_read(handle: Handle, buf: &mut [u8]) -> Result<usize>;
        fn __rust_bind_fs_file_write(handle: Handle, buf: &[u8]) -> Result<usize>;
        fn __rust_bind_fs_file_flush(handle: Handle) -> Result<()>;
        fn __rust_bind_fs_file_seek(handle: Handle, pos: io::SeekFrom) -> Result<u64>;
        fn __rust_bind_fs_file_fsync(handle: Handle) -> Result<()>;
        fn __rust_bind_fs_file_datasync(handle: Handle) -> Result<()>;
        fn __rust_bind_fs_file_truncate(handle: Handle, sz: u64) -> Result<()>;
        fn __rust_bind_fs_file_stat(handle: Handle) -> Result<FileAttr>;
        fn __rust_bind_fs_file_close(handle: Handle) -> Result<()>;
    }

    pub struct ReadDir;
    #[derive(Clone)]
    pub struct OpenOptions(u8/*, Mode*/);
    #[derive(Copy, Clone, PartialEq, Eq, Hash)]
    pub struct FileType;
    pub struct DirBuilder;
    pub struct DirEntry;
    #[derive(Debug)]
    pub struct File(Handle);
    #[derive(Clone)]
    pub struct FileAttr;
    #[derive(Clone, PartialEq, Eq, Debug)]
    pub struct FilePermissions(u32);

	struct OpenFlags;
	impl OpenFlags {
		const OPEN_FLAG_READ: u8 = 0b00000001;
		const OPEN_FLAG_WRITE: u8 = 0b00000010;
		const OPEN_FLAG_APPEND: u8 = 0b00000100;
		const OPEN_FLAG_TRUNCATE: u8 = 0b00001000;
		const OPEN_FLAG_CREATE: u8 = 0b00010000;
	}

    impl Iterator for ReadDir {
        type Item = Result<DirEntry>;
        fn next(&mut self) -> Option<Result<DirEntry>> { panic!() }
    }

    impl DirEntry {
        pub fn file_name(&self) -> Cow<OsStr> { panic!() }
        pub fn root(&self) -> &OsStr { panic!() }
        pub fn metadata(&self) -> Result<FileAttr> { panic!() }
        pub fn file_type(&self) -> Result<FileType> { panic!() }
        //pub fn ino(&self) -> INode { panic!() }
    }

    impl OpenOptions {
        pub fn new() -> Self { OpenOptions(0/*, 0*/) }

        pub fn read(&mut self, read: bool) { self.0 = self.0 | OpenFlags::OPEN_FLAG_READ }
        pub fn write(&mut self, write: bool) { self.0 = self.0 | OpenFlags::OPEN_FLAG_WRITE }
        pub fn append(&mut self, append: bool) { self.0 = self.0 | OpenFlags::OPEN_FLAG_APPEND }
        pub fn truncate(&mut self, truncate: bool) { self.0 = self.0 | OpenFlags::OPEN_FLAG_TRUNCATE }
        pub fn create(&mut self, create: bool) { self.0 = self.0 | OpenFlags::OPEN_FLAG_CREATE }
        //pub fn mode(&mut self, mode: Mode) { self.1 = mode }
    }

    impl FileType {
        pub fn is_dir(&self) -> bool { panic!() }
        pub fn is_file(&self) -> bool { panic!() }
        pub fn is_symlink(&self) -> bool { panic!() }
    }

    impl DirBuilder {
        pub fn new() -> Self { panic!() }
        pub fn mkdir(&self, p: &OsStr) -> Result<()> { panic!() }

        //pub fn set_mode(&mut self, mode: Mode) { panic!() }
    }

    impl File {
        pub fn open(path: &OsStr, opts: &OpenOptions) -> Result<Self> {
            unsafe { __rust_bind_fs_file_open(path, opts).map(File) }
        }

        pub fn open_c(path: &CStr, opts: &OpenOptions) -> Result<Self> {
			unimplemented!()
        }

        pub fn fsync(&self) -> Result<()> {
            unsafe { __rust_bind_fs_file_fsync(self.0) }
        }

        pub fn datasync(&self) -> Result<()> {
            unsafe { __rust_bind_fs_file_datasync(self.0) }
        }

        pub fn truncate(&self, sz: u64) -> Result<()> {
            unsafe { __rust_bind_fs_file_truncate(self.0, sz) }
        }

        pub fn file_attr(&self) -> Result<FileAttr> {
            unsafe { __rust_bind_fs_file_stat(self.0) }
        }

        pub fn read(&self, buf: &mut [u8]) -> Result<usize> {
            unsafe { __rust_bind_fs_file_read(self.0, buf) }
        }

        pub fn write(&self, buf: &[u8]) -> Result<usize> {
            unsafe { __rust_bind_fs_file_write(self.0, buf) }
        }

        pub fn flush(&self) -> Result<()> {
            unsafe { __rust_bind_fs_file_flush(self.0) }
        }

        pub fn seek(&self, pos: io::SeekFrom) -> Result<u64> {
            unsafe { __rust_bind_fs_file_seek(self.0, pos) }
        }
    }

    impl Drop for File {
        fn drop(&mut self) {
            let _ = unsafe { __rust_bind_fs_file_close(self.0) };
        }
    }

    impl_inner!(File(Handle): AsInner + FromInner + IntoInnerForget);

    impl FileAttr {
        pub fn size(&self) -> u64 { panic!() }
        pub fn perm(&self) -> FilePermissions { panic!() }
        pub fn file_type(&self) -> FileType { panic!() }
    }

    impl FilePermissions {
        pub fn readonly(&self) -> bool { panic!() }
        pub fn set_readonly(&mut self, readonly: bool) { panic!() }
        //pub fn mode(&self) -> Mode { panic!() }
    }

	pub fn unlink(p: &OsStr) -> Result<()> {
		unsafe { __rust_bind_fs_unlink(p) }
	}

	pub fn stat(p: &OsStr) -> Result<FileAttr> {
		unsafe { __rust_bind_fs_stat(p) }
	}

	pub fn lstat(p: &OsStr) -> Result<FileAttr> {
		unsafe { __rust_bind_fs_lstat(p) }
	}

	pub fn rename(from: &OsStr, to: &OsStr) -> Result<()> {
		unsafe { __rust_bind_fs_rename(from, to) }
	}

	pub const COPY_IMP: bool = true;
	pub fn copy(from: &OsStr, to: &OsStr) -> Result<u64> {
		unsafe { __rust_bind_fs_copy(from, to) }
	}

	pub fn link(src: &OsStr, dst: &OsStr) -> Result<()> {
		unsafe { __rust_bind_fs_link(src, dst) }
	}

	pub fn symlink(src: &OsStr, dst: &OsStr) -> Result<()> {
		unsafe { __rust_bind_fs_symlink(src, dst) }
	}

	pub fn readlink(p: &OsStr) -> Result<OsString> {
		unsafe { __rust_bind_fs_readlink(p) }
	}

	pub fn canonicalize(p: &OsStr) -> Result<OsString> {
		unsafe { __rust_bind_fs_canonicalize(p) }
	}

	pub fn rmdir(p: &OsStr) -> Result<()> {
		unsafe { __rust_bind_fs_rmdir(p) }
	}

	pub fn set_perm(p: &OsStr, perm: FilePermissions) -> Result<()> {
		unsafe { __rust_bind_fs_set_perm(p, perm) }
	}

	pub fn readdir(p: &OsStr) -> Result<ReadDir> { panic!() }
}

pub mod path {
    use ffi::OsStr;
	use path;

	#[inline]
	pub fn is_sep_byte(b: u8) -> bool {
		b == b'/'
	}

	#[inline]
	pub fn is_verbatim_sep(b: u8) -> bool {
		b == b'/'
	}

	pub const PREFIX_IMP: bool = false;

	pub fn parse_prefix(s: &OsStr) -> Option<path::Prefix> {
		None
	}

	pub const MAIN_SEP_STR: &'static str = "/";
	pub const MAIN_SEP: char = '/';
}

pub mod rt {
    use os::raw::c_char;

    extern {
        pub fn __rust_bind_rt_strlen(s: *const c_char) -> usize;
    }

	pub use sys::common::rt::{std_cleanup, min_stack};
    pub use self::__rust_bind_rt_strlen as strlen;

	pub unsafe fn run_main<R, F: FnOnce() -> R>(f: F, argc: isize, argv: *const *const u8) -> R { panic!() }
	pub unsafe fn run_thread<R, F: FnOnce() -> R>(f: F) -> R { panic!() }
	pub unsafe fn thread_cleanup() { panic!() }
	pub unsafe fn cleanup() { panic!() }
}

pub use sys::common::os_str::u8 as os_str;
