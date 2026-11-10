use super::assets;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use std::path::Path;
use std::time::Duration;

pub fn handle(mut stream: TcpStream, root: &Path) -> io::Result<()> {
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
    let line = read_request(&stream)?;
    let Some((method, path)) = parse_request(&line) else {
        return respond(
            &mut stream,
            "400 Bad Request",
            "text/plain",
            b"Bad request",
            false,
        );
    };
    route_request(&mut stream, root, method, path)
}

fn read_request(stream: &TcpStream) -> io::Result<String> {
    let mut line = String::new();
    BufReader::new(stream.take(8192)).read_line(&mut line)?;
    Ok(line)
}

fn parse_request(line: &str) -> Option<(&str, &str)> {
    let fields: Vec<_> = line.split_whitespace().collect();
    if fields.len() == 3 && line.ends_with('\n') {
        Some((fields[0], fields[1]))
    } else {
        None
    }
}

fn route_request(stream: &mut TcpStream, root: &Path, method: &str, path: &str) -> io::Result<()> {
    let head = method == "HEAD";
    if method != "GET" && !head {
        return respond(
            stream,
            "405 Method Not Allowed",
            "text/plain",
            b"Use GET or HEAD",
            false,
        );
    }
    match assets::resolve(root, path).and_then(|path| {
        std::fs::read(&path)
            .ok()
            .map(|body| (assets::content_type(&path), body))
    }) {
        Some((mime, body)) => respond(stream, "200 OK", mime, &body, head),
        None => respond(stream, "404 Not Found", "text/plain", b"Not found", head),
    }
}

fn respond(
    stream: &mut TcpStream,
    status: &str,
    mime: &str,
    body: &[u8],
    head: bool,
) -> io::Result<()> {
    write!(
        stream,
        "HTTP/1.1 {status}\r\nContent-Type: {mime}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    if !head {
        stream.write_all(body)?;
    }
    stream.flush()
}
