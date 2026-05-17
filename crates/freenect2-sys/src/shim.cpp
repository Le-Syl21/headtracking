// Method bodies + free-function definitions for the cxx shim. See `shim.h`.

#include "shim.h"
#include "freenect2-sys/src/lib.rs.h"

#include <libfreenect2/packet_pipeline.h>

#include <cstring>

namespace freenect2_shim {

RustLogger::RustLogger(libfreenect2::Logger::Level lvl) {
    level_ = lvl;
}

libfreenect2::Logger::Level RustLogger::level() const {
    return level_;
}

void RustLogger::log(libfreenect2::Logger::Level level,
                     const std::string &message) {
    // cxx's `rust::Str` is a non-owning UTF-8 view; libfreenect2 hands
    // us ASCII/UTF-8 messages, so the conversion is safe.
    freenect2_log_forward(static_cast<uint32_t>(level),
                          rust::Str(message.data(), message.size()));
}

void install_logger(uint32_t level) {
    // setGlobalLogger takes ownership and deletes any previously-
    // installed logger. Allocating a fresh RustLogger on every call
    // keeps the API idempotent — repeated calls just replace the
    // current logger.
    libfreenect2::setGlobalLogger(new RustLogger(
        static_cast<libfreenect2::Logger::Level>(level)));
}

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

bool RgbSink::onNewFrame(libfreenect2::Frame::Type type,
                         libfreenect2::Frame *frame) {
    if (type != libfreenect2::Frame::Color) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mu_);
    const size_t bytes =
        static_cast<size_t>(frame->width) * frame->height * frame->bytes_per_pixel;
    width_ = frame->width;
    height_ = frame->height;
    timestamp_ = frame->timestamp;
    data_.resize(bytes);
    std::memcpy(data_.data(), frame->data, bytes);
    has_new_.store(true, std::memory_order_release);
    return false;  // we copied; libfreenect2 keeps ownership.
}

bool RgbSink::poll(uint32_t &w, uint32_t &h, uint32_t &ts,
                   std::vector<uint8_t> &out) {
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
    holder->dev->setIrAndDepthFrameListener(&holder->depth_listener);
    holder->dev->setColorFrameListener(&holder->rgb_listener);
    return holder;
}

bool start_depth(Freenect2Dev &dev) {
    if (!dev.dev) return false;
    return dev.dev->startStreams(/*rgb=*/false, /*depth=*/true);
}

bool start_streams(Freenect2Dev &dev, bool rgb, bool depth) {
    if (!dev.dev) return false;
    return dev.dev->startStreams(rgb, depth);
}

bool stop_device(Freenect2Dev &dev) {
    if (!dev.dev) return false;
    return dev.dev->stop();
}

bool poll_depth(Freenect2Dev &dev, DepthFrame &out) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    std::vector<float> data;
    if (!dev.depth_listener.poll(w, h, ts, data)) {
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

bool poll_rgb(Freenect2Dev &dev, RgbFrame &out) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    std::vector<uint8_t> data;
    if (!dev.rgb_listener.poll(w, h, ts, data)) {
        return false;
    }
    out.width = w;
    out.height = h;
    out.timestamp_raw = ts;
    out.data.clear();
    out.data.reserve(data.size());
    for (uint8_t b : data) {
        out.data.push_back(b);
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
