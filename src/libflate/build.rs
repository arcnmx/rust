use std::path::PathBuf;
use std::env;

extern crate gcc;

fn main() {
    let sources = &[
        "../rt/miniz.c",
    ];

    let mut config = gcc::Config::new();
    config.cargo_metadata(false);
    compile_library("libminiz.a", sources, config);

    let out_dir = PathBuf::from(&env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-flags=-L native={}", out_dir.display());
}

fn compile_library(output: &str, files: &[&str], mut c: gcc::Config) {
    for f in files.iter() {
        c.file(*f);
    }
    c.compile(output)
}
