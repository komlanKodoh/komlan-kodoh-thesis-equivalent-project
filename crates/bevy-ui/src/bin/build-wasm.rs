use std::io;
use std::path::Path;
use std::process::Command;

fn main() -> io::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    require_wasm_toolchain()?;
    build_rust(&root)?;
    bind_javascript(&root)?;
    println!("Built wasm-interface in web/native/");
    Ok(())
}

fn require_wasm_toolchain() -> io::Result<()> {
    let output = Command::new("rustc")
        .args([
            "--print",
            "target-libdir",
            "--target",
            "wasm32-unknown-unknown",
        ])
        .output()?;
    let path = String::from_utf8_lossy(&output.stdout);
    if !output.status.success() || !Path::new(path.trim()).is_dir() {
        return Err(io::Error::other(
            "Install the WASM target: rustup target add wasm32-unknown-unknown",
        ));
    }
    let binding_tool = Command::new("wasm-bindgen").arg("--version").output();
    if !binding_tool.is_ok_and(|result| result.status.success()) {
        return Err(io::Error::other(
            "Install the bindings tool: cargo install wasm-bindgen-cli --locked",
        ));
    }
    Ok(())
}

fn build_rust(root: &Path) -> io::Result<()> {
    let status = Command::new(env!("CARGO"))
        .current_dir(root)
        .args([
            "build",
            "-p",
            "wasm-interface",
            "--lib",
            "--release",
            "--target",
            "wasm32-unknown-unknown",
        ])
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other("WebAssembly build failed"))
    }
}

fn bind_javascript(root: &Path) -> io::Result<()> {
    let source = "target/wasm32-unknown-unknown/release/wasm_interface.wasm";
    let status = Command::new("wasm-bindgen")
        .current_dir(root)
        .args([
            "--target",
            "web",
            "--out-dir",
            "web/native",
            "--out-name",
            "aether_control",
            source,
        ])
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other(
            "wasm-bindgen failed; install wasm-bindgen-cli",
        ))
    }
}
