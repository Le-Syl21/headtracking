// C++ shim header: defines the opaque types that the cxx bridge in `lib.rs`
// references. The full class layouts must live here (not in shim.cpp) because
// cxx generates `std::unique_ptr<T>::~unique_ptr()` at the bridge site, and
// that requires `sizeof(T)` to be known.

#pragma once

#include "rust/cxx.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace freenect2_shim {

// Forward declarations of cxx-generated shared types (defined in lib.rs.h).
struct DepthFrame;
struct IrCameraParams;

class DepthSink : public libfreenect2::FrameListener {
public:
    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame) override;

    // Drain the latest depth frame into `out` if a new one is available.
    bool poll(uint32_t &width, uint32_t &height, uint32_t &timestamp,
              std::vector<float> &data);

private:
    std::mutex mu_;
    std::atomic<bool> has_new_{false};
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t timestamp_ = 0;
    std::vector<float> data_;
};

struct Freenect2Ctx {
    libfreenect2::Freenect2 inner;
};

struct Freenect2Dev {
    libfreenect2::Freenect2Device *dev = nullptr;
    DepthSink listener;

    ~Freenect2Dev();

    // Disable copy: the device owns a USB session.
    Freenect2Dev() = default;
    Freenect2Dev(const Freenect2Dev &) = delete;
    Freenect2Dev &operator=(const Freenect2Dev &) = delete;
};

// Free functions exposed through the cxx bridge.
std::unique_ptr<Freenect2Ctx> new_context();
int32_t enumerate(Freenect2Ctx &ctx);
std::unique_ptr<Freenect2Dev> open_default(Freenect2Ctx &ctx);

bool start_depth(Freenect2Dev &dev);
bool stop_device(Freenect2Dev &dev);
bool poll_depth(Freenect2Dev &dev, DepthFrame &out);
IrCameraParams ir_params(const Freenect2Dev &dev);

}  // namespace freenect2_shim
