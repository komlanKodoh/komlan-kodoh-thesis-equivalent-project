#[path = "../server/assets.rs"]
mod assets;
#[path = "../server/http.rs"]
mod http;

use std::io;
use std::net::TcpListener;
use std::path::Path;
use std::process::Command;

fn main() -> io::Result<()> {
    let project = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    require_engine(&project)?;
    build_browser(&project)?;
    let address = "127.0.0.1:8080";
    let root = project.join("web/dist").canonicalize()?;
    let listener = TcpListener::bind(address)?;
    println!("Aether simulation: http://{address}");
    println!("Press Ctrl+C to stop.");
    for connection in listener.incoming() {
        match connection {
            Ok(stream) => {
                let root = root.clone();
                std::thread::spawn(move || {
                    if let Err(error) = http::handle(stream, &root) {
                        eprintln!("Request failed: {error}");
                    }
                });
            }
            Err(error) => eprintln!("Connection failed: {error}"),
        }
    }
    Ok(())
}

fn require_engine(project: &Path) -> io::Result<()> {
    let assets = ["aether_control.js", "aether_control_bg.wasm"];
    if assets
        .iter()
        .all(|name| project.join("web/native").join(name).is_file())
    {
        Ok(())
    } else {
        Err(io::Error::other(
            "Build the controller first: cargo run -p bevy-ui --bin build-wasm",
        ))
    }
}

fn build_browser(project: &Path) -> io::Result<()> {
    let status = Command::new(env!("CARGO"))
        .current_dir(project)
        .args(["run", "--quiet", "-p", "bevy-ui", "--bin", "build-web"])
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other("Browser build failed"))
    }
}
