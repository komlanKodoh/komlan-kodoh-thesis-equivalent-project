use std::io;
use std::path::Path;
use std::process::Command;

fn main() -> io::Result<()> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let status = Command::new("node")
        .current_dir(root)
        .args(["--experimental-strip-types", "web/build.ts"])
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(io::Error::other("TypeScript browser build failed"))
    }
}
