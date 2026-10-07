// Compiles kernels/*.cu to PTX when the `gpu` feature is on.
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=kernels/pathfinder.cu");
    println!("cargo:rerun-if-changed=build.rs");
    if std::env::var_os("CARGO_FEATURE_GPU").is_none() {
        return;
    }
    let out = std::env::var("OUT_DIR").unwrap();
    let nvcc = std::env::var("NVCC").unwrap_or_else(|_| "nvcc".into());
    // compute_75 PTX: JIT-compiled by the driver for this and newer GPUs.
    // -fmad=false keeps float arithmetic un-fused (deterministic costs).
    let status = Command::new(&nvcc)
        .args(["-ptx", "-O3", "-fmad=false", "-arch=compute_75",
               "kernels/pathfinder.cu", "-o"])
        .arg(format!("{out}/pathfinder.ptx"))
        .status()
        .expect("failed to run nvcc (set NVCC=/path/to/nvcc)");
    assert!(status.success(), "nvcc failed");
}
