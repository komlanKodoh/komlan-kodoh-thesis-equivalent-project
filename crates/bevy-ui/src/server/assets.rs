use std::path::{Component, Path, PathBuf};

pub fn resolve(root: &Path, request: &str) -> Option<PathBuf> {
    let path = request.split('?').next()?.strip_prefix('/')?;
    if path.contains(['%', '\\']) {
        return None;
    }
    let relative = Path::new(if path.is_empty() { "index.html" } else { path });
    if relative
        .components()
        .any(|part| !matches!(part, Component::Normal(_)))
    {
        return None;
    }
    let candidate = root.join(relative).canonicalize().ok()?;
    (candidate.starts_with(root) && candidate.is_file()).then_some(candidate)
}

pub fn content_type(path: &Path) -> &'static str {
    match path.extension().and_then(|value| value.to_str()) {
        Some("html") => "text/html; charset=utf-8",
        Some("css") => "text/css; charset=utf-8",
        Some("js") => "text/javascript; charset=utf-8",
        Some("ts") => "text/javascript; charset=utf-8",
        Some("wasm") => "application/wasm",
        Some("svg") => "image/svg+xml",
        Some("png") => "image/png",
        _ => "application/octet-stream",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../web")
            .canonicalize()
            .unwrap()
    }

    #[test]
    fn serves_entry_and_module_with_query() {
        assert_eq!(resolve(&root(), "/").unwrap(), root().join("index.html"));
        assert!(resolve(&root(), "/scripts/app.ts?v=1").is_some());
        assert_eq!(content_type(Path::new("control.wasm")), "application/wasm");
    }

    #[test]
    fn rejects_paths_outside_web_root() {
        for path in [
            "/../Cargo.toml",
            "/%2e%2e/Cargo.toml",
            "/scripts/../../Cargo.toml",
            "/scripts\\app.ts",
            "/missing.ts",
        ] {
            assert!(resolve(&root(), path).is_none(), "{path}");
        }
    }
}
