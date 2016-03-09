extern crate gcc;

use std::process::Command;
use std::path::PathBuf;
use std::{fs, env};

fn main() {
	let dir = env::current_dir().unwrap();
	let out_dir = PathBuf::from(&env::var_os("OUT_DIR").unwrap());

	let compiler_rt = dir.join("../../src/compiler-rt");

	let config = gcc::Config::new().get_compiler();
	let cflags = config.args().into_iter().cloned()
		.map(|c| c.into_string().unwrap())
		.fold(String::new(), |mut args, arg| {
			args.push_str(&arg);
			args.push(' ');
			args
		});

	let mut make = Command::new("make");
	make.current_dir(&out_dir)
		.arg("-C").arg(&compiler_rt)
		.arg(format!("ProjSrcRoot={}", compiler_rt.display()))
		.arg(format!("ProjObjRoot={}", out_dir.display()))
		.arg(format!("TargetTriple={}", env::var("TARGET").unwrap()))
		.arg(format!("CFLAGS={}", cflags))
		.arg(format!("CC={}", config.path().display()))
		.arg("triple-builtins");

	println!("{:?}", make);
	make.status().unwrap();

	fs::copy(&out_dir.join("triple/builtins/libcompiler_rt.a"), &out_dir.join("../../../deps/libcompiler-rt.a")).unwrap();
	fs::rename(&out_dir.join("triple/builtins/libcompiler_rt.a"), &out_dir.join("libcompiler_rt.a")).unwrap();

	println!("cargo:rustc-link-lib=static=compiler_rt");
	println!("cargo:rustc-flags=-L native={}", out_dir.display());
}
