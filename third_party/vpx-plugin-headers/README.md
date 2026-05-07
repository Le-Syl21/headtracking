# Vendored VPX plugin headers

Copies of the C plugin API headers from the upstream
[`vpinball`](https://github.com/vpinball/vpinball) repository,
license: GPLv3+ (matches our own GPL-3.0-or-later).

These are the bindgen targets — see `build.rs` and `wrapper.h` at the
repo root. Headers are copied verbatim:

| File                  | Upstream path                                       |
|-----------------------|-----------------------------------------------------|
| `MsgPlugin.h`         | `plugins/plugins/MsgPlugin.h`                       |
| `VPXPlugin.h`         | `plugins/plugins/VPXPlugin.h`                       |
| `LoggingPlugin.h`     | `plugins/plugins/LoggingPlugin.h`                   |
| `ControllerPlugin.h`  | `plugins/plugins/ControllerPlugin.h` (transitive)   |

## Why vendor

The build script's preferred path is `../vpinball/plugins/plugins/` —
that's a developer's local checkout of vpinball, kept up-to-date with
upstream. CI machines don't have that checkout; they fall back to this
in-tree copy.

When VPX's plugin API stabilises, we'll switch to whichever release tag
the headers ship at. Until then we refresh manually when upstream
changes — the headers explicitly warn the API is unstable.

## Refreshing

```sh
cp ../vpinball/plugins/plugins/{MsgPlugin,VPXPlugin,LoggingPlugin,ControllerPlugin}.h \
   third_party/vpx-plugin-headers/
```

Then rebuild and run the tests; the bindgen output regenerates
automatically because `build.rs` watches `wrapper.h`.
