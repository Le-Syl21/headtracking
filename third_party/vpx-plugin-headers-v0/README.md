# Vendored VPX plugin headers, legacy unversioned API

Verbatim copies of `plugins/plugins/{MsgPlugin,VPXPlugin,LoggingPlugin,ControllerPlugin}.h`
from [`vpinball`](https://github.com/vpinball/vpinball) at commit
`af26b2d93` (2026-08-20), the base of the 10.8.1-5436 pre-release.
License: GPLv3+.

This is the plugin API before vpinball `595d1fd` (2026-09-05): message
names without the `:1` suffix and dispatch tables without the leading
`int version`. `build.rs` binds only `MsgPluginAPI`, `VPXPluginAPI` and
`LoggingPluginAPI` from here (`src/plugin/vpx_sys_v0.rs`), so the plugin
can still load in hosts that predate the change. `src/plugin/host_api.rs`
detects the revision at load time and converts these tables to the
current layout.

Do not refresh these files: they are frozen on purpose. The current API
lives in `../vpx-plugin-headers/`.
