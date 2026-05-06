// Aggregate header consumed by bindgen (see build.rs).
// Lists every VPX plugin header we want bindings for.
//
// The headers live in `../vpinball/plugins/plugins/` by default; the path is
// resolved by build.rs and added as a clang `-I` flag, so simple includes work.

#include "MsgPlugin.h"
#include "VPXPlugin.h"
#include "LoggingPlugin.h"
