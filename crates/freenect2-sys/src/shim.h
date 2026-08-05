// C++ shim header: defines the opaque types that the cxx bridge in `lib.rs`
// references. The full class layouts must live here (not in shim.cpp) because
// cxx generates `std::unique_ptr<T>::~unique_ptr()` at the bridge site, and
// that requires `sizeof(T)` to be known.

#pragma once

#include "rust/cxx.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace freenect2_shim {

// Forward declarations of cxx-generated shared types (defined in lib.rs.h).
struct DepthFrame;
struct IrFrame;
struct RgbFrame;
struct IrCameraParams;
struct ColorCameraParams;
struct ColorPixel;

// The combined IR+Depth listener slot. libfreenect2 delivers both the IR and
// the Depth frame to a *single* FrameListener (setIrAndDepthFrameListener),
// tagged by Frame::Type — there is no separate IR listener to register. So one
// sink object keeps the latest of each in its own slot, and the Rust side sees
// three symmetric pipes (poll_depth / poll_ir / poll_rgb).
class IrDepthSink : public libfreenect2::FrameListener {
public:
    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

    // Drain the latest depth frame (float millimetres) if a new one arrived.
    bool poll_depth(uint32_t &width, uint32_t &height, uint32_t &timestamp,
                    std::vector<float> &data);

    // Drain the latest IR frame (float intensity, ~0..65535) if a new one
    // arrived. Same 512×424 geometry as depth.
    bool poll_ir(uint32_t &width, uint32_t &height, uint32_t &timestamp,
                 std::vector<float> &data);

private:
    std::mutex depth_mu_;
    std::atomic<bool> depth_new_{false};
    uint32_t depth_w_ = 0;
    uint32_t depth_h_ = 0;
    uint32_t depth_ts_ = 0;
    std::vector<float> depth_;

    std::mutex ir_mu_;
    std::atomic<bool> ir_new_{false};
    uint32_t ir_w_ = 0;
    uint32_t ir_h_ = 0;
    uint32_t ir_ts_ = 0;
    std::vector<float> ir_;
};

class RgbSink : public libfreenect2::FrameListener {
public:
    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

    bool poll(uint32_t &width, uint32_t &height, uint32_t &timestamp,
              std::vector<uint8_t> &data);

private:
    std::mutex mu_;
    std::atomic<bool> has_new_{false};
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t timestamp_ = 0;
    // Pixel layout matches libfreenect2's `Frame::BGRX` for the Kinect v2:
    // 4 bytes per pixel, channel order [B, G, R, X].
    std::vector<uint8_t> data_;
};

struct Freenect2Ctx {
    libfreenect2::Freenect2 inner;
};

struct Freenect2Dev {
    libfreenect2::Freenect2Device *dev = nullptr;
    IrDepthSink ir_depth_listener;
    RgbSink rgb_listener;

    ~Freenect2Dev();

    // Disable copy: the device owns a USB session.
    Freenect2Dev() = default;
    Freenect2Dev(const Freenect2Dev &) = delete;
    Freenect2Dev &operator=(const Freenect2Dev &) = delete;
};

// Owns a `libfreenect2::Registration` built from the device's factory IR +
// color intrinsics. Maps depth pixels onto the color image using libfreenect2's
// reverse-engineered depth↔color model — the proper fix for the IR-vs-RGB
// sensor parallax (~5 cm baseline, different FOV) that a naive resolution-ratio
// scale can't correct. `inner` is null if built before the device streamed its
// camera params (getColorCameraParams would be all-zero).
struct Registration {
    std::unique_ptr<libfreenect2::Registration> inner;
    // Scratch planes apply() insists on filling even though we only keep
    // bigdepth. Persistent members so the 30 Hz call doesn't allocate
    // ~1.7 MB per frame; sized lazily on first use.
    std::vector<unsigned char> undistorted_scratch;
    std::vector<unsigned char> registered_scratch;
};

// Subclass of libfreenect2's Logger that forwards each log call into
// Rust tracing. The bridge in `lib.rs` declares the Rust receiver
// `freenect2_log_forward(u32, &str)`. Heap-allocated and handed to
// `libfreenect2::setGlobalLogger`, which takes ownership and frees
// the previous logger when replaced.
class RustLogger : public libfreenect2::Logger {
public:
    explicit RustLogger(libfreenect2::Logger::Level lvl);
    libfreenect2::Logger::Level level() const override;
    void log(libfreenect2::Logger::Level level,
             const std::string &message) override;
};

// Free functions exposed through the cxx bridge.
void install_logger(uint32_t level);
std::unique_ptr<Freenect2Ctx> new_context();
int32_t enumerate(Freenect2Ctx &ctx);
std::unique_ptr<Freenect2Dev> open_default(Freenect2Ctx &ctx);

bool start_depth(Freenect2Dev &dev);
bool start_streams(Freenect2Dev &dev, bool rgb, bool depth);
bool stop_device(Freenect2Dev &dev);
bool poll_depth(Freenect2Dev &dev, DepthFrame &out);
bool poll_ir(Freenect2Dev &dev, IrFrame &out);
bool poll_rgb(Freenect2Dev &dev, RgbFrame &out);
IrCameraParams ir_params(const Freenect2Dev &dev);
ColorCameraParams color_params(const Freenect2Dev &dev);
std::unique_ptr<Registration> new_registration(const Freenect2Dev &dev);
ColorPixel map_depth_to_color(const Registration &reg, int32_t dx, int32_t dy,
                              float dz);
bool register_bigdepth(Registration &reg, rust::Slice<const uint8_t> rgb,
                       rust::Slice<const float> depth,
                       rust::Slice<float> bigdepth);

}  // namespace freenect2_shim
