// Method bodies + free-function definitions for the cxx shim. See `shim.h`.

#include "shim.h"
#include "freenect2-sys/src/lib.rs.h"

#include <libfreenect2/config.h>
#include <libfreenect2/packet_pipeline.h>

#include <cmath>
#include <cstring>
#include <limits>

namespace freenect2_shim {

namespace {
// Kinect v2 sensor geometry, fixed in hardware and hard-checked by
// libfreenect2's Registration::apply().
constexpr size_t kColorW = 1920;
constexpr size_t kColorH = 1080;
constexpr size_t kDepthW = 512;
constexpr size_t kDepthH = 424;
// apply() writes bigdepth as its internal filter map: the colour plane plus a
// one-row border top and bottom (filter_height_half = 1), i.e. 1920×1082
// floats. Colour row `y` therefore lives at bigdepth row `y + 1`.
constexpr size_t kBigDepthLen = kColorW * (kColorH + 2);
}  // namespace

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

bool IrDepthSink::onNewFrame(libfreenect2::Frame::Type type,
                             libfreenect2::Frame *frame) {
    const size_t pixels = static_cast<size_t>(frame->width) * frame->height;
    if (type == libfreenect2::Frame::Depth) {
        std::lock_guard<std::mutex> lock(depth_mu_);
        // Landing on a slot the consumer never read means this sensor frame
        // dies here. Counted, because the alternative is a log where a 30 Hz
        // sensor and a 10 Hz reader look the same.
        if (depth_new_.load(std::memory_order_relaxed)) {
            depth_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        depth_received_.fetch_add(1, std::memory_order_relaxed);
        depth_w_ = frame->width;
        depth_h_ = frame->height;
        depth_ts_ = frame->timestamp;
        depth_.resize(pixels);
        // libfreenect2 hands us f32 millimeters (0.0 = no data).
        std::memcpy(depth_.data(), frame->data, pixels * sizeof(float));
        depth_new_.store(true, std::memory_order_release);
    } else if (type == libfreenect2::Frame::Ir) {
        std::lock_guard<std::mutex> lock(ir_mu_);
        if (ir_new_.load(std::memory_order_relaxed)) {
            ir_dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        ir_received_.fetch_add(1, std::memory_order_relaxed);
        ir_w_ = frame->width;
        ir_h_ = frame->height;
        ir_ts_ = frame->timestamp;
        ir_.resize(pixels);
        // IR is f32 intensity in roughly [0, 65535].
        std::memcpy(ir_.data(), frame->data, pixels * sizeof(float));
        ir_new_.store(true, std::memory_order_release);
    }
    return false;  // we copied; libfreenect2 keeps ownership of the buffer.
}

bool IrDepthSink::poll_depth_into(uint32_t &w, uint32_t &h, uint32_t &ts,
                                  float *dst, size_t capacity) {
    if (!depth_new_.load(std::memory_order_acquire)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(depth_mu_);
    if (!depth_new_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (depth_.size() > capacity) {
        return false;
    }
    w = depth_w_;
    h = depth_h_;
    ts = depth_ts_;
    std::memcpy(dst, depth_.data(), depth_.size() * sizeof(float));
    depth_new_.store(false, std::memory_order_release);
    return true;
}

bool IrDepthSink::poll_ir_into(uint32_t &w, uint32_t &h, uint32_t &ts,
                               float *dst, size_t capacity) {
    if (!ir_new_.load(std::memory_order_acquire)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(ir_mu_);
    if (!ir_new_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (ir_.size() > capacity) {
        return false;
    }
    w = ir_w_;
    h = ir_h_;
    ts = ir_ts_;
    std::memcpy(dst, ir_.data(), ir_.size() * sizeof(float));
    ir_new_.store(false, std::memory_order_release);
    return true;
}

bool RgbSink::onNewFrame(libfreenect2::Frame::Type type,
                         libfreenect2::Frame *frame) {
    if (type != libfreenect2::Frame::Color) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mu_);
    if (has_new_.load(std::memory_order_relaxed)) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
    }
    received_.fetch_add(1, std::memory_order_relaxed);
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

bool RgbSink::poll_into(uint32_t &w, uint32_t &h, uint32_t &ts, uint8_t *dst,
                        size_t capacity) {
    if (!has_new_.load(std::memory_order_acquire)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mu_);
    if (!has_new_.load(std::memory_order_relaxed)) {
        return false;
    }
    if (data_.size() > capacity) {
        return false;
    }
    w = width_;
    h = height_;
    ts = timestamp_;
    std::memcpy(dst, data_.data(), data_.size());
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
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    // Prefer the OpenCL depth pipeline. The Kinect v2 phase-unwrap +
    // bilateral/edge-aware filtering is what pins the CPU pipeline at
    // ~260% (and drops USB depth packets when it can't keep up); OpenCL
    // runs it on the GPU instead. `openDefaultDevice` takes ownership of
    // the pipeline even on failure, so a null return means the GPU path
    // is already freed and we can safely retry with the CPU pipeline
    // (missing ICD, no usable device, etc.).
    {
        libfreenect2::PacketPipeline *gpu = new libfreenect2::OpenCLPacketPipeline();
        holder->dev = ctx.inner.openDefaultDevice(gpu);
        if (holder->dev) {
            holder->gpu_pipeline = true;
            holder->dev->setIrAndDepthFrameListener(&holder->ir_depth_listener);
            holder->dev->setColorFrameListener(&holder->rgb_listener);
            return holder;
        }
    }
#endif
    // CPU fallback (also the only path on non-OpenCL builds).
    libfreenect2::PacketPipeline *pipeline = new libfreenect2::CpuPacketPipeline();
    // openDefaultDevice takes ownership of the pipeline, even on failure.
    holder->dev = ctx.inner.openDefaultDevice(pipeline);
    if (!holder->dev) {
        return nullptr;
    }
    holder->dev->setIrAndDepthFrameListener(&holder->ir_depth_listener);
    holder->dev->setColorFrameListener(&holder->rgb_listener);
    return holder;
}

const char *depth_pipeline(const Freenect2Dev &dev) {
    return dev.gpu_pipeline ? "OpenCL" : "CPU";
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

bool poll_depth_into(Freenect2Dev &dev, rust::Slice<float> out, FrameMeta &meta) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    if (!dev.ir_depth_listener.poll_depth_into(w, h, ts, out.data(), out.size())) {
        return false;
    }
    meta.width = w;
    meta.height = h;
    meta.timestamp_raw = ts;
    return true;
}

bool poll_ir_into(Freenect2Dev &dev, rust::Slice<float> out, FrameMeta &meta) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    if (!dev.ir_depth_listener.poll_ir_into(w, h, ts, out.data(), out.size())) {
        return false;
    }
    meta.width = w;
    meta.height = h;
    meta.timestamp_raw = ts;
    return true;
}

bool poll_rgb_into(Freenect2Dev &dev, rust::Slice<uint8_t> out, FrameMeta &meta) {
    if (!dev.dev) return false;
    uint32_t w = 0, h = 0, ts = 0;
    if (!dev.rgb_listener.poll_into(w, h, ts, out.data(), out.size())) {
        return false;
    }
    meta.width = w;
    meta.height = h;
    meta.timestamp_raw = ts;
    return true;
}

StreamStats stream_stats(const Freenect2Dev &dev) {
    StreamStats s{};
    s.rgb_received = dev.rgb_listener.received();
    s.rgb_dropped = dev.rgb_listener.dropped();
    s.depth_received = dev.ir_depth_listener.depth_received();
    s.depth_dropped = dev.ir_depth_listener.depth_dropped();
    s.ir_received = dev.ir_depth_listener.ir_received();
    s.ir_dropped = dev.ir_depth_listener.ir_dropped();
    return s;
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

ColorCameraParams color_params(const Freenect2Dev &dev) {
    ColorCameraParams r{};
    if (!dev.dev) return r;
    auto p = const_cast<libfreenect2::Freenect2Device *>(dev.dev)->getColorCameraParams();
    r.fx = p.fx;
    r.fy = p.fy;
    r.cx = p.cx;
    r.cy = p.cy;
    r.shift_d = p.shift_d;
    r.shift_m = p.shift_m;
    r.mx_x3y0 = p.mx_x3y0;
    r.mx_x0y3 = p.mx_x0y3;
    r.mx_x2y1 = p.mx_x2y1;
    r.mx_x1y2 = p.mx_x1y2;
    r.mx_x2y0 = p.mx_x2y0;
    r.mx_x0y2 = p.mx_x0y2;
    r.mx_x1y1 = p.mx_x1y1;
    r.mx_x1y0 = p.mx_x1y0;
    r.mx_x0y1 = p.mx_x0y1;
    r.mx_x0y0 = p.mx_x0y0;
    r.my_x3y0 = p.my_x3y0;
    r.my_x0y3 = p.my_x0y3;
    r.my_x2y1 = p.my_x2y1;
    r.my_x1y2 = p.my_x1y2;
    r.my_x2y0 = p.my_x2y0;
    r.my_x0y2 = p.my_x0y2;
    r.my_x1y1 = p.my_x1y1;
    r.my_x1y0 = p.my_x1y0;
    r.my_x0y1 = p.my_x0y1;
    r.my_x0y0 = p.my_x0y0;
    return r;
}

bool register_bigdepth(Registration &reg, rust::Slice<const uint8_t> rgb,
                       rust::Slice<const float> depth,
                       rust::Slice<float> bigdepth) {
    if (!reg.inner) {
        return false;
    }
    // libfreenect2's apply() silently returns on any dimension mismatch, so
    // validate here — otherwise a wrong-sized buffer would look like success
    // while leaving stale data behind. Sizes are fixed by the sensor.
    if (rgb.size() != kColorW * kColorH * 4 || depth.size() != kDepthW * kDepthH ||
        bigdepth.size() != kBigDepthLen) {
        return false;
    }
    // `Frame(w, h, bpp, data)` with non-null `data` does NOT take ownership
    // (it leaves `rawdata` null, so ~Frame() deletes nothing). apply() only
    // reads rgb/depth, hence the const_cast onto libfreenect2's non-const API.
    libfreenect2::Frame rgb_f(kColorW, kColorH, 4,
                              const_cast<unsigned char *>(rgb.data()));
    libfreenect2::Frame depth_f(
        kDepthW, kDepthH, 4,
        reinterpret_cast<unsigned char *>(const_cast<float *>(depth.data())));
    libfreenect2::Frame bigdepth_f(
        kColorW, kColorH + 2, 4,
        reinterpret_cast<unsigned char *>(bigdepth.data()));
    // apply() requires both scratch outputs even though we only want bigdepth.
    // Persistent members (lazily sized): at 30 Hz, per-call vectors were
    // ~50 MB/s of pure allocator churn.
    reg.undistorted_scratch.resize(kDepthW * kDepthH * 4);
    reg.registered_scratch.resize(kDepthW * kDepthH * 4);
    libfreenect2::Frame undistorted_f(kDepthW, kDepthH, 4,
                                      reg.undistorted_scratch.data());
    libfreenect2::Frame registered_f(kDepthW, kDepthH, 4,
                                     reg.registered_scratch.data());
    // enable_filter MUST be true: bigdepth doubles as the filter map, and the
    // non-filtered path never writes it.
    reg.inner->apply(&rgb_f, &depth_f, &undistorted_f, &registered_f, true,
                     &bigdepth_f, nullptr);
    return true;
}

bool depth_window_min(Registration &reg, rust::Slice<const float> depth,
                      int32_t center_x, int32_t center_y, int32_t half,
                      rust::Slice<float> out) {
    if (!reg.inner || depth.size() != kDepthW * kDepthH || half < 0) {
        return false;
    }
    const size_t side = static_cast<size_t>(2 * half + 1);
    if (out.size() != side * side) {
        return false;
    }
    // Undistort once into the same scratch plane the full apply() uses: the
    // per-pixel apply() below indexes the *undistorted* grid.
    reg.undistorted_scratch.resize(kDepthW * kDepthH * 4);
    libfreenect2::Frame depth_f(
        kDepthW, kDepthH, 4,
        reinterpret_cast<unsigned char *>(const_cast<float *>(depth.data())));
    libfreenect2::Frame undistorted_f(kDepthW, kDepthH, 4,
                                      reg.undistorted_scratch.data());
    reg.inner->undistortDepth(&depth_f, &undistorted_f);
    const float *undistorted =
        reinterpret_cast<const float *>(reg.undistorted_scratch.data());

    for (float &v : out) {
        v = std::numeric_limits<float>::infinity();
    }

    // libfreenect2 splats every depth sample over a 5×3 colour window
    // (filter_width_half = 2, filter_height_half = 1) and keeps the nearest z
    // per colour pixel; `bigdepth` *is* that filter map. Reproduce it here,
    // clipped to the requested window, so the values match what bigdepth
    // would have held — for a fraction of the work: no 8.3 MB infinity fill,
    // no 3.3 M scattered writes, no registered image we throw away.
    const int32_t x0 = center_x - half;
    const int32_t y0 = center_y - half;
    const int32_t side_i = 2 * half + 1;
    for (size_t dy = 0; dy < kDepthH; ++dy) {
        for (size_t dx = 0; dx < kDepthW; ++dx) {
            const float z = undistorted[dy * kDepthW + dx];
            if (z <= 0.0f) {
                continue;
            }
            float cxf = 0.0f, cyf = 0.0f;
            reg.inner->apply(static_cast<int>(dx), static_cast<int>(dy), z, cxf,
                             cyf);
            // Same rounding as the bulk path (which folds the +0.5 into
            // color_cx and precomputes the y offset as an int).
            const int32_t cx = static_cast<int32_t>(cxf + 0.5f);
            const int32_t cy = static_cast<int32_t>(cyf + 0.5f);
            // Reject before touching the window: the colour frame is far
            // wider than the depth frustum, so most samples land nowhere near
            // a given head.
            if (cx + 2 < x0 || cx - 2 >= x0 + side_i || cy + 1 < y0 ||
                cy - 1 >= y0 + side_i) {
                continue;
            }
            for (int32_t r = cy - 1; r <= cy + 1; ++r) {
                if (r < y0 || r >= y0 + side_i) {
                    continue;
                }
                float *row = out.data() +
                             static_cast<size_t>(r - y0) * static_cast<size_t>(side_i);
                for (int32_t c = cx - 2; c <= cx + 2; ++c) {
                    if (c < x0 || c >= x0 + side_i) {
                        continue;
                    }
                    float &slot = row[c - x0];
                    if (z < slot) {
                        slot = z;
                    }
                }
            }
        }
    }
    return true;
}

std::unique_ptr<Registration> new_registration(const Freenect2Dev &dev) {
    auto reg = std::make_unique<Registration>();
    if (!dev.dev) {
        return reg;  // inner stays null
    }
    auto *d = const_cast<libfreenect2::Freenect2Device *>(dev.dev);
    reg->inner = std::make_unique<libfreenect2::Registration>(
        d->getIrCameraParams(), d->getColorCameraParams());
    return reg;
}

std::unique_ptr<Registration> new_registration_from_params(IrCameraParams ir,
                                                           ColorCameraParams color) {
    auto reg = std::make_unique<Registration>();
    libfreenect2::Freenect2Device::IrCameraParams i{};
    i.fx = ir.fx; i.fy = ir.fy; i.cx = ir.cx; i.cy = ir.cy;
    i.k1 = ir.k1; i.k2 = ir.k2; i.k3 = ir.k3; i.p1 = ir.p1; i.p2 = ir.p2;
    libfreenect2::Freenect2Device::ColorCameraParams c{};
    c.fx = color.fx; c.fy = color.fy; c.cx = color.cx; c.cy = color.cy;
    c.shift_d = color.shift_d;
    c.shift_m = color.shift_m;
    c.mx_x3y0 = color.mx_x3y0;
    c.mx_x0y3 = color.mx_x0y3;
    c.mx_x2y1 = color.mx_x2y1;
    c.mx_x1y2 = color.mx_x1y2;
    c.mx_x2y0 = color.mx_x2y0;
    c.mx_x0y2 = color.mx_x0y2;
    c.mx_x1y1 = color.mx_x1y1;
    c.mx_x1y0 = color.mx_x1y0;
    c.mx_x0y1 = color.mx_x0y1;
    c.mx_x0y0 = color.mx_x0y0;
    c.my_x3y0 = color.my_x3y0;
    c.my_x0y3 = color.my_x0y3;
    c.my_x2y1 = color.my_x2y1;
    c.my_x1y2 = color.my_x1y2;
    c.my_x2y0 = color.my_x2y0;
    c.my_x0y2 = color.my_x0y2;
    c.my_x1y1 = color.my_x1y1;
    c.my_x1y0 = color.my_x1y0;
    c.my_x0y1 = color.my_x0y1;
    c.my_x0y0 = color.my_x0y0;
    reg->inner = std::make_unique<libfreenect2::Registration>(i, c);
    return reg;
}

ColorPixel map_depth_to_color(const Registration &reg, int32_t dx, int32_t dy,
                              float dz) {
    ColorPixel out{};
    out.x = 0.0f;
    out.y = 0.0f;
    out.valid = false;
    if (!reg.inner) {
        return out;
    }
    // apply() returns the color-image pixel coordinate (0..1919, 0..1079)
    // despite the header calling it "normalized" — the implementation yields
    // pixels. Points with no valid mapping come back as ±inf / NaN.
    float cx = 0.0f, cy = 0.0f;
    reg.inner->apply(dx, dy, dz, cx, cy);
    if (std::isfinite(cx) && std::isfinite(cy)) {
        out.x = cx;
        out.y = cy;
        out.valid = true;
    }
    return out;
}

}  // namespace freenect2_shim
