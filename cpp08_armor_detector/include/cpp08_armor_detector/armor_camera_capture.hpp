#ifndef ARMOR_CAMERA_CAPTURE_HPP
#define ARMOR_CAMERA_CAPTURE_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <opencv2/opencv.hpp>

#if defined(ARMOR_TRACKER_USE_MVSDK)
#include "CameraApi.h"
#endif

namespace armor_camera
{

// 相机配置参数
struct CameraConfig
{
    int exposure_time_us = 2000;      // 曝光时间（微秒），仅 MVSDK 有效
    bool auto_exposure = false;       // 自动曝光，false 表示手动
    // 可扩展其他参数，如分辨率、帧率等
};

// ---------------------- 后端接口 ----------------------
class ICameraBackend
{
public:
    virtual ~ICameraBackend() = default;
    virtual bool open(const CameraConfig& cfg) = 0;
    virtual bool read(cv::Mat& frame) = 0;
    virtual void release() = 0;
};

// ---------------------- MVSDK 后端 ----------------------
#if defined(ARMOR_TRACKER_USE_MVSDK)
class MvCameraBackend : public ICameraBackend
{
public:
    ~MvCameraBackend() override;
    bool open(const CameraConfig& cfg) override;
    bool read(cv::Mat& frame) override;
    void release() override;

private:
    CameraHandle camera_handle_{};
    tSdkCameraCapbility capability_{};
    std::vector<unsigned char> rgb_buffer_;
    bool initialized_ = false;
};
#endif // ARMOR_TRACKER_USE_MVSDK

// ---------------------- OpenCV 后端 ----------------------
class OpenCvCameraBackend : public ICameraBackend
{
public:
    ~OpenCvCameraBackend() override;
    bool open(const CameraConfig& cfg) override;
    bool read(cv::Mat& frame) override;
    void release() override;

private:
    cv::VideoCapture capture_;
    bool initialized_ = false;
};

// ---------------------- 统一封装类 ----------------------
class ArmorCameraCapture
{
public:
    explicit ArmorCameraCapture(const CameraConfig& cfg = CameraConfig());
    ~ArmorCameraCapture();

    ArmorCameraCapture(const ArmorCameraCapture&) = delete;
    ArmorCameraCapture& operator=(const ArmorCameraCapture&) = delete;

    bool open();
    bool read(cv::Mat& frame);
    void release();
    void setConfig(const CameraConfig& cfg);

private:
    std::unique_ptr<ICameraBackend> backend_;
    CameraConfig config_;
};

} // namespace armor_camera

#endif // ARMOR_CAMERA_CAPTURE_HPP