fn main() {
    // Give the Windows .exe the webcam icon (shows in Explorer and the
    // taskbar). Build scripts run on the host, so gate on the TARGET os.
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        let mut res = winresource::WindowsResource::new();
        res.set_icon("assets/icon.ico");
        if let Err(e) = res.compile() {
            println!("cargo:warning=winresource failed ({e}) — exe ships without icon");
        }
    }
    println!("cargo:rerun-if-changed=assets/icon.ico");
}
