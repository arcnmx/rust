use std::process::Command;
use std::path::PathBuf;
use std::env;

extern crate gcc;

fn main() {
    let dir = env::current_dir().unwrap();
    let llvmdeps = PathBuf::from(&env::var_os("CFG_LLVM_LINKAGE_FILE").unwrap());
    let llvm_config = PathBuf::from(&env::var_os("CFG_LLVM_CONFIG").unwrap());
    let std_cpp = PathBuf::from(&env::var_os("CFG_LLVM_STDCPP").unwrap());
    let std_cpp_static = PathBuf::from(&env::var_os("CFG_LLVM_STDCPP_STATIC").unwrap());

    let sources = &[
        "../rustllvm/ArchiveWrapper.cpp",
        "../rustllvm/ExecutionEngineWrapper.cpp",
        "../rustllvm/PassWrapper.cpp",
        "../rustllvm/RustWrapper.cpp",
    ];

    let mklldeps = dir.join("../etc/mklldeps.py");

    run(Command::new("python2")
        .arg(mklldeps)
        .arg(llvmdeps)
        .arg("")
        .arg(std_cpp_static)
        .arg(&llvm_config)
        .arg(std_cpp));

    println!("cargo:rustc-link-lib=ffi");
    println!("cargo:rustc-link-search=native={}", String::from_utf8(Command::new(&llvm_config).arg("--libdir").output().unwrap().stdout).unwrap());
    println!("cargo:rustc-link-search=native={}", "/usr/lib");

    let mut config = gcc::Config::new();
    let cxxflags = Command::new(llvm_config).arg("--cxxflags").output().unwrap().stdout;
    let cxxflags = String::from_utf8(cxxflags).unwrap();
    for flag in cxxflags.split(' ') {
        if !flag.starts_with("-W") {
            let flag = flag.trim();
            if !flag.is_empty() {
                config.flag(flag.trim());
            }
        }
    }
    config.cargo_metadata(false);
    compile_library("librustllvm.a", sources, config);

    let out_dir = PathBuf::from(&env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-flags=-L native={}", out_dir.display());
}

fn run(cmd: &mut Command) {
    println!("running: {:?}", cmd);
    cmd.status().unwrap();
}

fn compile_library(output: &str, files: &[&str], mut c: gcc::Config) {
    for f in files.iter() {
        c.file(*f);
    }
    c.compile(output)
}
