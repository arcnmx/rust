extern crate gcc;

use std::path::PathBuf;
use std::env;

fn main() {
    let sources = &[
        "../rt/hoedown/src/autolink.c",
        "../rt/hoedown/src/buffer.c",
        "../rt/hoedown/src/document.c",
        "../rt/hoedown/src/escape.c",
        "../rt/hoedown/src/html.c",
        "../rt/hoedown/src/html_blocks.c",
        "../rt/hoedown/src/html_smartypants.c",
        "../rt/hoedown/src/stack.c",
        "../rt/hoedown/src/version.c",
    ];

    let mut config = gcc::Config::new();
    config.cargo_metadata(false);
    compile_library("libhoedown.a", sources, config);

    let out_dir = PathBuf::from(&env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-flags=-L native={}", out_dir.display());
}

fn compile_library(output: &str, files: &[&str], mut c: gcc::Config) {
    for f in files.iter() {
        c.file(*f);
    }
    c.compile(output)
}
