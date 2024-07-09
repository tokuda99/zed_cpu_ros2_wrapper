#pragma once

#include <cv_bridge/cv_bridge.h>

#include <boost/filesystem.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <zed-open-capture/videocapture.hpp>
#include <zed_cpu_ros2_wrapper/calibration.hpp>

#include <memory>
#include <string>
#include <thread>

using namespace sl_oc::video;
typedef struct {
    uint32_t width;
    uint32_t height;
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;
    cv::Mat map_x;
    cv::Mat map_y;
} CalibrationParams;

sensor_msgs::msg::CameraInfo CvtCalibParamsToCameraInfo(
    const CalibrationParams &calib_params, const std::string &frame_id,
    const rclcpp::Time &stamp);

class ZedParams {
  private:
    RESOLUTION resolution_;
    FPS fps_;
    void cvtStringToEnum(
        const std::string resolution_s = "", const std::string fps_s = "") {
        if (resolution_s == "HD2K") {
            resolution_ = RESOLUTION::HD2K;
        } else if (resolution_s == "HD1080") {
            resolution_ = RESOLUTION::HD1080;
        } else if (resolution_s == "HD720") {
            resolution_ = RESOLUTION::HD720;
        } else if (resolution_s == "VGA") {
            resolution_ = RESOLUTION::VGA;
        } else if (resolution_s == "") {
            ;
        } else {
            resolution_ = RESOLUTION::HD1080;
        }
        std::string fps_s_temp = fps_s;
        if (fps_s_temp == "100") {
            fps_ = FPS::FPS_100;
            if (resolution_ != RESOLUTION::VGA) fps_s_temp = "60";
        }
        if (fps_s_temp == "60") {
            fps_ = FPS::FPS_60;
            if (resolution_ != RESOLUTION::VGA &&
                resolution_ != RESOLUTION::HD720)
                fps_s_temp = "30";
        }
        if (fps_s_temp == "30") {
            fps_ = FPS::FPS_30;
            if (resolution_ != RESOLUTION::VGA &&
                resolution_ != RESOLUTION::HD720 &&
                resolution_ != RESOLUTION::HD1080)
                fps_s_temp = "15";
        }
        if (fps_s_temp == "15") {
            fps_ = FPS::FPS_15;
        }
    }
    void cvtEnumToString(std::string &resolution_s, std::string &fps_s) {
        switch (resolution_) {
            case RESOLUTION::HD2K:
                resolution_s = "HD2K";
                break;
            case RESOLUTION::HD1080:
                resolution_s = "HD1080";
                break;
            case RESOLUTION::HD720:
                resolution_s = "HD720";
                break;
            case RESOLUTION::VGA:
                resolution_s = "VGA";
                break;
            default:
                resolution_s = "unknown";
                break;
        }
        switch (fps_) {
            case FPS::FPS_100:
                fps_s = "100";
                break;
            case FPS::FPS_60:
                fps_s = "60";
                break;
            case FPS::FPS_30:
                fps_s = "30";
                break;
            case FPS::FPS_15:
                fps_s = "15";
                break;
            default:
                fps_s = "unknown";
                break;
        }
    }

  public:
    ZedParams(std::string resolution_s = "HD1080", std::string fps_s = "15") {
        cvtStringToEnum(resolution_s, fps_s);
    }
    RESOLUTION getResolution() { return resolution_; }
    FPS getFps() { return fps_; }
    std::string getResolutionStr() {
        std::string resolution_s, fps_s;
        cvtEnumToString(resolution_s, fps_s);
        return resolution_s;
    }
    std::string getFpsStr() {
        std::string resolution_s, fps_s;
        cvtEnumToString(resolution_s, fps_s);
        return fps_s;
    }
    void setResolution(std::string resolution_s) {
        cvtStringToEnum(resolution_s, "");
    }
    void setFps(std::string fps_s) { cvtStringToEnum("", fps_s); }
};

// using namespace sl_oc::video;
class ZedCpuRos2Wrapper : public rclcpp::Node {
  private:
    int device_id_, serial_number_;
    ZedParams zed_params_;
    std::string left_camera_frame_id_, right_camera_frame_id_;
    CalibrationParams left_calib_params_, right_calib_params_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        left_camerainfo_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        right_camerainfo_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
        left_compressed_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
        right_compressed_image_pub_;

    std::shared_ptr<camera_info_manager::CameraInfoManager>
        left_camera_info_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager>
        right_camera_info_manager_;

    std::shared_ptr<sl_oc::video::VideoCapture> video_cap_ptr_;
    std::thread capture_thread_;
    std::atomic<bool> canceled_;

    double publish_rate_;
    rclcpp::TimerBase::SharedPtr image_pub_timer_;
    bool publish_next_frame_;

    bool use_camera_buffer_timestamps_;
    bool compressed_image_transport_;
    bool initCalibrationParms();
    bool openCamera() noexcept;
    void cameraSpinner();

  public:
    explicit ZedCpuRos2Wrapper(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
        int device_id = -1);
    ~ZedCpuRos2Wrapper();
};
