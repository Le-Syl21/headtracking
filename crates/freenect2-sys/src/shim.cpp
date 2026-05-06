// Method bodies + free-function definitions for the cxx shim. See `shim.h`.

#include "shim.h"
#include "freenect2-sys/src/lib.rs.h"

#include <libfreenect2/packet_pipeline.h>

#include <cstring>

namespace freenect2_shim {

bool DepthSink::onNewFrame(libfreenect2::Frame::Type type,
                           libfreenect2::Frame *frame) {
    if (type != libfreenect2::Frame::Depth) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mu_);
    const size_t pixels = static_cast<size_t>(frame->width) * frame->height;
    width_ = frame->width;
    height_ = frame->height;
    timestamp_ = frame->timestamp;
    data_.resize(pixels);
    // libfreenect2 hands us f32 millimeters (0.0 = no data).
    std::memcpy(data_.data(), frame->data, pixels * sizeof(float));
    has_new_.store(true, std::memory_order_release);
    return false;  // we copied; libfreenect2 keeps ownership of the buffer.
}

bool DepthSink::poll(uint32_t &w, uint32_t &h, uint32_t &ts,
                     std::vector<float> &out) {
    if (!has_new_.load(std::memory_order_acquire)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mu_);
    if (!has_new_.load(std::memory_order_relaxed)) {
        return false;
    }
    w = width_;
    h = height_;
    ts = timestamp_;
    out = data_;
    has_new_.store(false, std::memory_order_release);
    return true;
}

Freenect2Dev::~Freenect2Dev() {
    if (dev) {
        dev->stop();
        dev->close();
        // Freenect2Device pointers are owned by Freenect2's internal
        // implementation; libfreenect2 deletes them when the parent
        // context tears down. Don't double-free here.
    }
}

std::unique_ptr<Freenect2Ctx> new_context() {
    return std::make_unique<Freenect2Ctx>();
}

int32_t enumerate(Freenect2Ctx &ctx) {
    return ctx.inner.enumerateDevices();
}

std::unique_ptr<Freenect2Dev> open_default(Freenect2Ctx &ctx) {
    auto holder = std::make_unique<Freenect2Dev>();
    libfreenect2::PacketPipeline *pipeline = new libfreenect2::CpuPacketPipeline();
    // openDefaultDevice takes ownership of the pipeline, even on failure.
    holder->dev = ctx.inner.openDefaultDevice(pipeline);
    if (!holder->dev) {
        return nullptr;
    }
    holder->dev->setIrAndDepthFrameListener(&holder->listener);
    return holder;
}

bool start_depth(Freenect2Dev &dev) {
    if (!dev.dev) return false;
    return dev.dev->startStreams(/*rgb=*/false, /*depth=*/true);
}

bool stop_device(Freenect2Dev &dev) {
    if (!dev.dev) return false;
    return dev.dev->stop();
}

bool poll_depth(Freenect2Dev &dev, DepthFrame &out) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    std::vector<float> data;
    if (!dev.listener.poll(w, h, ts, data)) {
        return false;
    }
    out.width = w;
    out.height = h;
    out.timestamp_raw = ts;
    out.data.clear();
    out.data.reserve(data.size());
    for (float v : data) {
        out.data.push_back(v);
    }
    return true;
}

IrCameraParams ir_params(const Freenect2Dev &dev) {
    IrCameraParams r{};
    if (!dev.dev) return r;
    auto p = const_cast<libfreenect2::Freenect2Device *>(dev.dev)->getIrCameraParams();
    r.fx = p.fx;
    r.fy = p.fy;
    r.cx = p.cx;
    r.cy = p.cy;
    r.k1 = p.k1;
    r.k2 = p.k2;
    r.k3 = p.k3;
    r.p1 = p.p1;
    r.p2 = p.p2;
    return r;
}

}  // namespace freenect2_shim
