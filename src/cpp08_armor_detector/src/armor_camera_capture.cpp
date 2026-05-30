#include "cpp08_armor_detector/armor_camera_capture.hpp"

namespace armor_camera
{

// ==================== MVSDK 后端实现 ====================
#if defined(ARMOR_TRACKER_USE_MVSDK)

MvCameraBackend::~MvCameraBackend()
{
    release();
}

bool MvCameraBackend::open(const CameraConfig& cfg)
{
    if (initialized_)
        return true;

    // SDK 初始化
    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS)
        return false;

    std::vector<tSdkCameraDevInfo> camera_list(4);
    INT camera_count = static_cast<INT>(camera_list.size());
    if (CameraEnumerateDevice(camera_list.data(), &camera_count) != CAMERA_STATUS_SUCCESS || camera_count <= 0)
        return false;

    // 打开第一台相机
    if (CameraInit(&camera_list[0], -1, -1, &camera_handle_) != CAMERA_STATUS_SUCCESS)
        return false;

    initialized_ = true;

    // 获取相机能力
    if (CameraGetCapability(camera_handle_, &capability_) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }

    // 设置输出格式（彩色或黑白）
    const bool is_mono = capability_.sIspCapacity.bMonoSensor;
    const UINT output_format = is_mono ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_BGR8;
    if (CameraSetIspOutFormat(camera_handle_, output_format) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }

    // 分配缓冲区
    const std::size_t max_w = static_cast<std::size_t>(capability_.sResolutionRange.iWidthMax);
    const std::size_t max_h = static_cast<std::size_t>(capability_.sResolutionRange.iHeightMax);
    const std::size_t channels = is_mono ? 1U : 3U;
    rgb_buffer_.resize(max_w * max_h * channels);
    if (rgb_buffer_.empty())
    {
        release();
        return false;
    }

    // 设置曝光（手动/自动）
    if (!cfg.auto_exposure)
    {
        CameraSetAeState(camera_handle_, FALSE);
        CameraSetExposureTime(camera_handle_, static_cast<double>(cfg.exposure_time_us));
    }
    else
    {
        CameraSetAeState(camera_handle_, TRUE);
    }

    // 开始采集
    if (CameraPlay(camera_handle_) != CAMERA_STATUS_SUCCESS)
    {
        release();
        return false;
    }

    return true;
}

bool MvCameraBackend::read(cv::Mat& frame)
{
    if (!initialized_)
        return false;

    tSdkFrameHead frame_info{};
    BYTE* raw_buffer = nullptr;
    if (CameraGetImageBuffer(camera_handle_, &frame_info, &raw_buffer, 1000) != CAMERA_STATUS_SUCCESS)
        return false;

    // 处理图像（转成 RGB 或 Mono）
    if (CameraImageProcess(camera_handle_, raw_buffer, rgb_buffer_.data(), &frame_info) != CAMERA_STATUS_SUCCESS)
    {
        CameraReleaseImageBuffer(camera_handle_, raw_buffer);
        return false;
    }
    CameraReleaseImageBuffer(camera_handle_, raw_buffer);

    const int image_type = (frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) ? CV_8UC1 : CV_8UC3;
    frame = cv::Mat(frame_info.iHeight, frame_info.iWidth, image_type, rgb_buffer_.data()).clone();
    if (frame.empty())
        return false;

    // 统一转为 BGR 彩色（可根据需要调整）
    if (image_type == CV_8UC1)
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

    return true;
}

void MvCameraBackend::release()
{
    if (initialized_)
    {
        CameraUnInit(camera_handle_);
        initialized_ = false;
    }
    rgb_buffer_.clear();
}

#endif // ARMOR_TRACKER_USE_MVSDK

// ==================== OpenCV 后端实现 ====================

OpenCvCameraBackend::~OpenCvCameraBackend()
{
    release();
}

bool OpenCvCameraBackend::open(const CameraConfig& /*cfg*/)
{
    if (initialized_)
        return true;

    if (!capture_.open(0))
        return false;

    initialized_ = true;
    return true;
}

bool OpenCvCameraBackend::read(cv::Mat& frame)
{
    if (!initialized_)
        return false;

    capture_.read(frame);
    return !frame.empty();
}

void OpenCvCameraBackend::release()
{
    if (initialized_)
    {
        capture_.release();
        initialized_ = false;
    }
}

// ==================== 统一封装类实现 ====================

ArmorCameraCapture::ArmorCameraCapture(const CameraConfig& cfg)
    : config_(cfg)
{
#if defined(ARMOR_TRACKER_USE_MVSDK)
    backend_ = std::make_unique<MvCameraBackend>();
#else
    backend_ = std::make_unique<OpenCvCameraBackend>();
#endif
}

ArmorCameraCapture::~ArmorCameraCapture()
{
    release();
}

bool ArmorCameraCapture::open()
{
    return backend_ && backend_->open(config_);
}

bool ArmorCameraCapture::read(cv::Mat& frame)
{
    return backend_ && backend_->read(frame);
}

void ArmorCameraCapture::release()
{
    if (backend_)
        backend_->release();
}

void ArmorCameraCapture::setConfig(const CameraConfig& cfg)
{
    config_ = cfg;
}

} // namespace armor_camera