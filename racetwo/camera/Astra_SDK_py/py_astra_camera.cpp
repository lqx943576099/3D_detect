#include <astra/astra.hpp>
#include <astra/DeviceController.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

namespace py = pybind11;

class AstraCamera {
public:
    AstraCamera()
    {
        // 延迟初始化，调用 start() 时再启动相机
        started_ = false;
        mirror_ = false;
        holeFillEnabled_ = false;
        holeFillIterations_ = 1;
        // 默认分辨率与帧率（可在 start() 前通过 set_*_mode 修改）
        desiredColorWidth_ = 640;
        desiredColorHeight_ = 480;
        desiredColorFps_ = 30;
        desiredDepthWidth_ = 640;
        desiredDepthHeight_ = 480;
        desiredDepthFps_ = 30;
        // 默认输出：全部开启（可在构造后 start() 前通过 set_outputs 控制）
        wantColor_ = true;
        wantDepth_ = true;
        wantShaded_ = true;
        wantCloud_ = true;
    }

    // 允许在构造时决定需要的输出（无效参数将抛出异常）
    AstraCamera(bool wantColor, bool wantDepth, bool wantShaded, bool wantCloud)
        : AstraCamera()
    {
        set_outputs(wantColor, wantDepth, wantShaded, wantCloud);
    }

    // 新增：允许在创建对象后设置是否镜像（可在 start() 前调用）
    void set_mirror(bool m)
    {
        mirror_ = m;
    }

    // 启用/禁用空洞填充（仅 CPU 端后处理）
    void set_hole_filling(bool enable, int iterations = 1)
    {
        holeFillEnabled_ = enable;
        holeFillIterations_ = std::max(0, iterations);
    }

    // 设备侧畸变矫正开关：status 通常为 0/1/2（关/开/自动，取决于设备固件）
    void set_distortion(int status)
    {
        if (!started_ || !streamSet_) {
            throw std::runtime_error("camera is not started");
        }
        astra::DeviceController dc(*streamSet_);
        dc.enable_distortion(status);
    }

    // 在 start() 过程中（各流 start 之前）设置畸变矫正开关
    void set_distortion_on_start(int status)
    {
        distortionOnStart_ = status; // -1 表示不触发
    }

    ~AstraCamera()
    {
        try { stop(); } catch (...) {}
    }

    // 显式启动相机
    void start()
    {
        if (started_) { return; }

        astra::initialize();

        streamSet_ = std::make_unique<astra::StreamSet>();
        reader_ = std::make_unique<astra::StreamReader>(streamSet_->create_reader());

        // 按需创建各流句柄（仅在需要时创建和配置）
        if (wantDepth_ || wantCloud_ || wantShaded_) {
            depthStream_ = std::make_unique<astra::DepthStream>(reader_->stream<astra::DepthStream>());
        }
        if (wantColor_) {
            colorStream_ = std::make_unique<astra::ColorStream>(reader_->stream<astra::ColorStream>());
        }
        if (wantCloud_ || wantShaded_) {
            pointStream_ = std::make_unique<astra::PointStream>(reader_->stream<astra::PointStream>());
        }

        // 应用指定的分辨率与帧率（默认 640x480@30）
        if (depthStream_) {
            astra::ImageStreamMode depthMode;
            depthMode.set_width(desiredDepthWidth_);
            depthMode.set_height(desiredDepthHeight_);
            depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
            depthMode.set_fps(desiredDepthFps_);
            depthStream_->set_mode(depthMode);
        }

        if (colorStream_) {
            astra::ImageStreamMode colorMode;
            colorMode.set_width(desiredColorWidth_);
            colorMode.set_height(desiredColorHeight_);
            colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
            colorMode.set_fps(desiredColorFps_);
            colorStream_->set_mode(colorMode);
        }

        // 镜像设置：对色彩和深度同时应用相同的左右镜像，保证两者方向一致
        if (mirror_) {
            if (colorStream_) { colorStream_->enable_mirroring(true); }
            if (depthStream_) { depthStream_->enable_mirroring(true); }
        } else {
            if (colorStream_) { colorStream_->enable_mirroring(false); }
            if (depthStream_) { depthStream_->enable_mirroring(false); }
        }

        // 对齐深度到彩色，使得彩色像素与点云索引一致
        if (depthStream_ && colorStream_) {
            depthStream_->enable_registration(true);
        }

        // 在开启各流之前下发畸变设置（部分设备仅在未开流时接受）
        if (distortionOnStart_ >= 0) {
            try {
                astra::DeviceController dc(*streamSet_);
                dc.enable_distortion(distortionOnStart_);
            } catch (...) {
                // 忽略设备不支持的情况
            }
        }

        if (colorStream_) { colorStream_->start(); }
        if (depthStream_) { depthStream_->start(); }
        if (pointStream_) { pointStream_->start(); }

        started_ = true;
    }

    // 显式关闭相机
    void stop()
    {
        if (!started_) { return; }

        // 重置流和读取器
        pointStream_.reset();
        colorStream_.reset();
        depthStream_.reset();
        reader_.reset();
        streamSet_.reset();

        astra::terminate();
        started_ = false;
    }

    // 获取相机内参与左右相机外参（Orbbec 提供的 r2l 变换）
    py::dict get_camera_params() const
    {
        if (!started_ || !streamSet_) {
            throw std::runtime_error("camera is not started");
        }

        astra::DeviceController dc(*streamSet_);
        orbbec_camera_params params{};
        dc.get_orbbec_camera_params(params);

        py::dict left;
        left["fx"] = params.l_intr_p[0];
        left["fy"] = params.l_intr_p[1];
        left["cx"] = params.l_intr_p[2];
        left["cy"] = params.l_intr_p[3];
        left["k"] = py::make_tuple(
            params.l_k[0], params.l_k[1], params.l_k[2], params.l_k[3], params.l_k[4]
        );

        py::dict right;
        right["fx"] = params.r_intr_p[0];
        right["fy"] = params.r_intr_p[1];
        right["cx"] = params.r_intr_p[2];
        right["cy"] = params.r_intr_p[3];
        right["k"] = py::make_tuple(
            params.r_k[0], params.r_k[1], params.r_k[2], params.r_k[3], params.r_k[4]
        );

        // r2l: 右相机到左相机的外参（R、t），行优先 3x3
        py::list R;
        for (int r = 0; r < 3; ++r) {
            py::list row;
            for (int c = 0; c < 3; ++c) {
                row.append(params.r2l_r[r * 3 + c]);
            }
            R.append(row);
        }
        py::tuple t = py::make_tuple(params.r2l_t[0], params.r2l_t[1], params.r2l_t[2]);

        py::dict out;
        out["left"] = left;
        out["right"] = right;
        out["R_r2l"] = R;
        out["t_r2l"] = t;
        return out;
    }

    // 无参版本：按对象配置返回帧
    py::tuple get_frames()
    {
        return get_frames(wantColor_, wantDepth_, wantShaded_, wantCloud_);
    }

    // 兼容版本：仍然保留带参接口
    py::tuple get_frames(bool want_color,
                         bool want_depth,
                         bool want_shaded,
                         bool want_cloud)
    {
        if (!want_color && !want_depth && !want_cloud) {
            warn("get_frames: no output requested (all flags are false)");
            return py::tuple(0);
        }

        if (!started_) {
            warn("get_frames: camera is not started; call start() first");
            return py::tuple(0);
        }
        astra::Frame frame = reader_->get_latest_frame();

        // 预先获取帧（仅在需要时验证）
        const auto colorFrame = frame.get<astra::ColorFrame>();
        const auto depthFrame = frame.get<astra::DepthFrame>();
        const auto pointFrame = frame.get<astra::PointFrame>();

        // 构建动态返回
        std::vector<py::object> outputs;
        outputs.reserve(3);

        // color 处理（无改动）
        if (want_color) {
            if (!colorFrame.is_valid()) {
                warn("get_frames: ColorFrame is invalid");
                return py::tuple(0);
            }
            const int cw = colorFrame.width();
            const int ch = colorFrame.height();
            const astra::RgbPixel* csrc = colorFrame.data();
            py::array_t<uint8_t> color_arr({ch, cw, 3});
            auto cbuf = color_arr.mutable_unchecked<3>();
            for (int y = 0; y < ch; ++y) {
                for (int x = 0; x < cw; ++x) {
                    const astra::RgbPixel& p = csrc[y * cw + x];
                    cbuf(y, x, 0) = p.r;
                    cbuf(y, x, 1) = p.g;
                    cbuf(y, x, 2) = p.b;
                }
            }
            outputs.emplace_back(std::move(color_arr));
        }

        // depth 部分：只在需要 shaded 时才要求 pointFrame，有助于避免不必要的依赖
        if (want_depth) {
            if (!depthFrame.is_valid()) {
                warn("get_frames: DepthFrame is invalid");
                return py::tuple(0);
            }

            // A) 深度图（always produce）
            const int dw = depthFrame.width();
            const int dh = depthFrame.height();
            const int16_t* dsrc = depthFrame.data();
            py::array_t<uint16_t> depth_arr({dh, dw});
            auto dbuf = depth_arr.mutable_unchecked<2>();
            for (int y = 0; y < dh; ++y) {
                for (int x = 0; x < dw; ++x) {
                    const int16_t z = dsrc[y * dw + x];
                    dbuf(y, x) = static_cast<uint16_t>(z < 0 ? 0 : z);
                }
            }

            // 深度空洞填充
            if (holeFillEnabled_ && holeFillIterations_ > 0) {
                uint16_t* depthPtr = reinterpret_cast<uint16_t*>(depth_arr.mutable_data());
                fill_depth_holes_inplace(depthPtr, dw, dh, holeFillIterations_);
            }

            // B) 仅当用户请求 shaded 时才计算着色图（需要 pointFrame）
            if (want_shaded) {
                if (!pointFrame.is_valid()) {
                    warn("get_frames: PointFrame is invalid (required for shaded depth)");
                    return py::tuple(0);
                }

                const int pw = pointFrame.width();
                const int ph = pointFrame.height();
                const astra::Vector3f* psrc = pointFrame.data();
                py::array_t<uint8_t> shaded({ph, pw, 3});

                // 计算法线
                std::vector<astra::Vector3f> normals(static_cast<size_t>(pw) * ph, astra::Vector3f::zero());
                for (int y = 1; y < ph - 1; ++y) {
                    for (int x = 1; x < pw - 1; ++x) {
                        const astra::Vector3f& p  = psrc[y * pw + x];
                        const astra::Vector3f& pl = psrc[y * pw + (x - 1)];
                        const astra::Vector3f& pr = psrc[y * pw + (x + 1)];
                        const astra::Vector3f& pu = psrc[(y - 1) * pw + x];
                        const astra::Vector3f& pd = psrc[(y + 1) * pw + x];

                        if (p.z != 0 && pl.z != 0 && pr.z != 0 && pu.z != 0 && pd.z != 0) {
                            const astra::Vector3f vr = pr - p;
                            const astra::Vector3f vd = pd - p;
                            const astra::Vector3f vl = pl - p;
                            const astra::Vector3f vu = pu - p;

                            astra::Vector3f n = vd.cross(vr);
                            n += vl.cross(vd);
                            n += vu.cross(vl);
                            n += vr.cross(vu);

                            normals[y * pw + x] = astra::Vector3f::normalize(n);
                        } else {
                            normals[y * pw + x] = astra::Vector3f::zero();
                        }
                    }
                }

                // 3x3 盒式模糊
                std::vector<astra::Vector3f> blurred(normals.size(), astra::Vector3f::zero());
                for (int y = 1; y < ph - 1; ++y) {
                    for (int x = 1; x < pw - 1; ++x) {
                        astra::Vector3f sum = astra::Vector3f::zero();
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dx = -1; dx <= 1; ++dx) {
                                sum += normals[(y + dy) * pw + (x + dx)];
                            }
                        }
                        blurred[y * pw + x] = astra::Vector3f::normalize(sum);
                    }
                }

                const astra::Vector3f lightDir(0.44022f, -0.17609f, 0.88045f);
                const astra::RgbPixel lightColor(210, 210, 210);
                const astra::RgbPixel ambientColor(30, 30, 30);

                auto out = shaded.mutable_unchecked<3>();
                for (int y = 0; y < ph; ++y) {
                    for (int x = 0; x < pw; ++x) {
                        const astra::Vector3f& P = psrc[y * pw + x];
                        const astra::Vector3f& N = blurred[y * pw + x];

                        if (P.z == 0) {
                            out(y, x, 0) = 0; out(y, x, 1) = 0; out(y, x, 2) = 0;
                            continue;
                        }

                        const float diffuse = std::max(0.0f, N.dot(lightDir));
                        astra::RgbPixel diffuseColor(0, 0, 0);
                        if (diffuse > 0.0f) {
                            diffuseColor.r = static_cast<uint8_t>(lightColor.r * diffuse);
                            diffuseColor.g = static_cast<uint8_t>(lightColor.g * diffuse);
                            diffuseColor.b = static_cast<uint8_t>(lightColor.b * diffuse);
                        }

                        const float depth_m = P.z;
                        const float fade = 1.0f - 0.6f * std::max(0.0f, std::min(1.0f, (depth_m - 400.0f) / 3200.0f));

                        const int r = static_cast<int>(fade * (ambientColor.r + diffuseColor.r));
                        const int g = static_cast<int>(fade * (ambientColor.g + diffuseColor.g));
                        const int b = static_cast<int>(fade * (ambientColor.b + diffuseColor.b));

                        out(y, x, 0) = static_cast<uint8_t>(std::max(0, std::min(255, r)));
                        out(y, x, 1) = static_cast<uint8_t>(std::max(0, std::min(255, g)));
                        out(y, x, 2) = static_cast<uint8_t>(std::max(0, std::min(255, b)));
                    }
                }

                // 返回 depth + shaded
                py::tuple depth_tuple(2);
                depth_tuple[0] = std::move(depth_arr);
                depth_tuple[1] = std::move(shaded);
                outputs.emplace_back(std::move(depth_tuple));
            } else {
                // 仅返回 depth
                outputs.emplace_back(std::move(depth_arr));
            }
        }

        // cloud 部分（无改动）
        if (want_cloud) {
            if (!pointFrame.is_valid()) {
                warn("get_frames: PointFrame is invalid");
                return py::tuple(0);
            }
            const int pw = pointFrame.width();
            const int ph = pointFrame.height();
            const astra::Vector3f* psrc = pointFrame.data();
            py::array_t<float> cloud_arr({ph, pw, 3});
            auto pbuf = cloud_arr.mutable_unchecked<3>();
            for (int y = 0; y < ph; ++y) {
                for (int x = 0; x < pw; ++x) {
                    const astra::Vector3f& v = psrc[y * pw + x];
                    pbuf(y, x, 0) = v.x;
                    pbuf(y, x, 1) = v.y;
                    pbuf(y, x, 2) = v.z;
                }
            }

            // 点云空洞填充（对 z==0 的点用邻域均值插值）
            if (holeFillEnabled_ && holeFillIterations_ > 0) {
                float* cloudPtr = reinterpret_cast<float*>(cloud_arr.mutable_data());
                fill_cloud_holes_inplace(cloudPtr, pw, ph, holeFillIterations_);
            }
            outputs.emplace_back(std::move(cloud_arr));
        }

        // 构造 py::tuple 返回
        py::tuple result(outputs.size());
        for (size_t i = 0; i < outputs.size(); ++i) {
            result[i] = outputs[i];
        }
        return result;
    }

    // 在 start() 前设置输出内容。若已启动，抛异常提示重启。
    void set_outputs(bool wantColor, bool wantDepth, bool wantShaded, bool wantCloud)
    {
        if (started_) {
            throw std::runtime_error("set_outputs must be called before start(); stop() then reconfigure");
        }
        // shaded/cloud 依赖深度/点云：若请求 shaded 或 cloud，则隐含需要 depth
        if (wantShaded || wantCloud) {
            wantDepth = true;
        }
        wantColor_ = wantColor;
        wantDepth_ = wantDepth;
        wantShaded_ = wantShaded;
        wantCloud_ = wantCloud;
    }

    // 配置彩色流模式（width/height/fps）。若相机已启动，将重配彩色流。
    void set_color_mode(int width, int height, int fps = 30)
    {
        if (width <= 0 || height <= 0 || fps <= 0) {
            throw std::invalid_argument("set_color_mode: width/height/fps must be positive");
        }
        desiredColorWidth_ = width;
        desiredColorHeight_ = height;
        desiredColorFps_ = fps;

        if (started_ && colorStream_) {
            reconfigure_color_stream_();
        }
    }

    // 配置深度流模式（width/height/fps）。若相机已启动，将重配深度与点云流。
    void set_depth_mode(int width, int height, int fps = 30)
    {
        if (width <= 0 || height <= 0 || fps <= 0) {
            throw std::invalid_argument("set_depth_mode: width/height/fps must be positive");
        }
        desiredDepthWidth_ = width;
        desiredDepthHeight_ = height;
        desiredDepthFps_ = fps;

        if (started_ && depthStream_) {
            reconfigure_depth_and_point_streams_();
        }
    }

    // 同时配置彩色与深度（便捷方法）。
    void set_resolution(int width, int height, int fps = 30)
    {
        set_color_mode(width, height, fps);
        set_depth_mode(width, height, fps);
    }
    

private:
    bool started_ {false};
    bool mirror_ {false}; 
    bool holeFillEnabled_ {false};
    int holeFillIterations_ {1};
    int distortionOnStart_ {-1};
    // 输出选择
    bool wantColor_ {true};
    bool wantDepth_ {true};
    bool wantShaded_ {true};
    bool wantCloud_ {true};
    int desiredColorWidth_ {640};
    int desiredColorHeight_ {480};
    int desiredColorFps_ {30};
    int desiredDepthWidth_ {640};
    int desiredDepthHeight_ {480};
    int desiredDepthFps_ {30};
    std::unique_ptr<astra::StreamSet> streamSet_;
    std::unique_ptr<astra::StreamReader> reader_;
    std::unique_ptr<astra::DepthStream> depthStream_;
    std::unique_ptr<astra::ColorStream> colorStream_;
    std::unique_ptr<astra::PointStream> pointStream_;

    static void fill_depth_holes_inplace(uint16_t* depth, int width, int height, int iterations)
    {
        if (!depth || width <= 2 || height <= 2 || iterations <= 0) { return; }
        std::vector<uint16_t> buffer(static_cast<size_t>(width) * height);
        for (int it = 0; it < iterations; ++it) {
            std::memcpy(buffer.data(), depth, buffer.size() * sizeof(uint16_t));
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    const size_t idx = static_cast<size_t>(y) * width + x;
                    if (buffer[idx] != 0) { continue; }
                    int sum = 0; int cnt = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) { continue; }
                            const uint16_t v = buffer[static_cast<size_t>(y + dy) * width + (x + dx)];
                            if (v != 0) { sum += v; ++cnt; }
                        }
                    }
                    if (cnt > 0) {
                        depth[idx] = static_cast<uint16_t>(sum / cnt);
                    }
                }
            }
        }
    }

    static void fill_cloud_holes_inplace(float* cloud, int width, int height, int iterations)
    {
        if (!cloud || width <= 2 || height <= 2 || iterations <= 0) { return; }
        const size_t num = static_cast<size_t>(width) * height;
        std::vector<float> buffer(num * 3);
        for (int it = 0; it < iterations; ++it) {
            std::memcpy(buffer.data(), cloud, buffer.size() * sizeof(float));
            for (int y = 1; y < height - 1; ++y) {
                for (int x = 1; x < width - 1; ++x) {
                    const size_t i3 = (static_cast<size_t>(y) * width + x) * 3;
                    const float z = buffer[i3 + 2];
                    if (z != 0.0f && std::isfinite(z)) { continue; }
                    float sx = 0.0f, sy = 0.0f, sz = 0.0f; int cnt = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) { continue; }
                            const size_t j3 = (static_cast<size_t>(y + dy) * width + (x + dx)) * 3;
                            const float nz = buffer[j3 + 2];
                            if (nz != 0.0f && std::isfinite(nz)) {
                                sx += buffer[j3 + 0];
                                sy += buffer[j3 + 1];
                                sz += buffer[j3 + 2];
                                ++cnt;
                            }
                        }
                    }
                    if (cnt > 0) {
                        cloud[i3 + 0] = sx / cnt;
                        cloud[i3 + 1] = sy / cnt;
                        cloud[i3 + 2] = sz / cnt;
                    }
                }
            }
        }
    }

    void warn(const char* message) const
    {
        try {
            py::module_::import("warnings").attr("warn")(message);
        } catch (...) {
            // fallback: ignore if warnings module not available
        }
    }

    void reconfigure_color_stream_()
    {
        try {
            if (!colorStream_) { return; }
            const bool wasMirrored = mirror_;
            colorStream_->stop();
            astra::ImageStreamMode colorMode;
            colorMode.set_width(desiredColorWidth_);
            colorMode.set_height(desiredColorHeight_);
            colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
            colorMode.set_fps(desiredColorFps_);
            colorStream_->set_mode(colorMode);
            colorStream_->enable_mirroring(wasMirrored);
            colorStream_->start();
        } catch (...) {
            warn("set_color_mode: failed to reconfigure running color stream; values will apply on next start()");
        }
    }

    void reconfigure_depth_and_point_streams_()
    {
        try {
            if (!depthStream_) { return; }
            const bool wasMirrored = mirror_;
            depthStream_->stop();
            if (pointStream_) { pointStream_->stop(); }
            astra::ImageStreamMode depthMode;
            depthMode.set_width(desiredDepthWidth_);
            depthMode.set_height(desiredDepthHeight_);
            depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
            depthMode.set_fps(desiredDepthFps_);
            depthStream_->set_mode(depthMode);
            depthStream_->enable_mirroring(wasMirrored);
            depthStream_->enable_registration(true);
            depthStream_->start();
            if (pointStream_) { pointStream_->start(); }
        } catch (...) {
            warn("set_depth_mode: failed to reconfigure running depth/point streams; values will apply on next start()");
        }
    }
};

PYBIND11_MODULE(astra_py, m) {
    m.doc() = "Astra camera control module (pybind11)";

    py::class_<AstraCamera>(m, "AstraCamera")
        .def(py::init<>())
        .def(py::init<bool,bool,bool,bool>(),
             py::arg("want_color"), py::arg("want_depth"), py::arg("want_shaded"), py::arg("want_cloud"))
        .def("start", &AstraCamera::start, "Start the Astra camera")
        .def("stop", &AstraCamera::stop, "Stop the Astra camera")
        .def("set_mirror", &AstraCamera::set_mirror, "Enable or disable mirroring before start()")
        .def("set_hole_filling", &AstraCamera::set_hole_filling,
             py::arg("enable"), py::arg("iterations") = 1,
             "Enable/disable simple CPU hole filling for depth and cloud (iterations: default 1)")
        .def("set_outputs", &AstraCamera::set_outputs,
             py::arg("want_color"), py::arg("want_depth"), py::arg("want_shaded"), py::arg("want_cloud"),
             "Select outputs before start(); only required streams will be opened.")
        .def("set_color_mode", &AstraCamera::set_color_mode,
             py::arg("width"), py::arg("height"), py::arg("fps") = 30,
             "Set color stream mode (width, height, fps). Applies immediately if running, else on start().")
        .def("set_depth_mode", &AstraCamera::set_depth_mode,
             py::arg("width"), py::arg("height"), py::arg("fps") = 30,
             "Set depth stream mode (width, height, fps). Applies immediately if running, else on start().")
        .def("set_resolution", &AstraCamera::set_resolution,
             py::arg("width"), py::arg("height"), py::arg("fps") = 30,
             "Convenience: set both color and depth modes to the same resolution and fps.")
        .def("set_distortion", &AstraCamera::set_distortion,
             py::arg("status"),
             "Enable device-side distortion correction: 0=off, 1=on, 2=auto (device-dependent)")
        .def("set_distortion_on_start", &AstraCamera::set_distortion_on_start,
             py::arg("status"),
             "Set device-side distortion mode to be applied during start() before streams are opened")
        .def("get_camera_params", &AstraCamera::get_camera_params,
             "Get intrinsics (fx,fy,cx,cy,k) for left/right and extrinsics R_r2l,t_r2l")
        .def("get_frames", py::overload_cast<>(&AstraCamera::get_frames))
        .def("get_frames", py::overload_cast<bool,bool,bool,bool>(&AstraCamera::get_frames),
             py::arg("want_color") = true,
             py::arg("want_depth") = true,
             py::arg("want_shaded") = true,
             py::arg("want_cloud") = true,
             "Get selected frames (color, depth, [depth_shaded], cloud) from the same frame as a tuple, in that order");
}


