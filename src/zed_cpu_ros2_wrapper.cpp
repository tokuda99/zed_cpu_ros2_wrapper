#include <zed_cpu_ros2_wrapper/zed_cpu_ros2_wrapper.hpp>

sensor_msgs::msg::CameraInfo CvtCalibParamsToCameraInfo(
    const CalibrationParams &calib_params, const std::string &frame_id = "",
    const rclcpp::Time &timestamp = rclcpp::Time(0)) {
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = frame_id;
    camera_info.header.stamp = timestamp;
    camera_info.width = calib_params.width;
    camera_info.height = calib_params.height;
    camera_info.distortion_model = "plumb_bob";
    camera_info.d.resize(5);
    for (int i = 0; i < 5; i++) {
        camera_info.d[i] = calib_params.dist_coeffs.at<double>(i);
    }
    camera_info.k.fill(0.0);
    camera_info.k[0] = calib_params.fx;
    camera_info.k[2] = calib_params.cx;
    camera_info.k[4] = calib_params.fy;
    camera_info.k[5] = calib_params.cy;
    camera_info.k[8] = 1.0;

    camera_info.r.fill(0.0);
    camera_info.r[0] = calib_params.rectification_matrix.at<double>(0, 0);
    camera_info.r[1] = calib_params.rectification_matrix.at<double>(0, 1);
    camera_info.r[2] = calib_params.rectification_matrix.at<double>(0, 2);
    camera_info.r[3] = calib_params.rectification_matrix.at<double>(1, 0);
    camera_info.r[4] = calib_params.rectification_matrix.at<double>(1, 1);
    camera_info.r[5] = calib_params.rectification_matrix.at<double>(1, 2);
    camera_info.r[6] = calib_params.rectification_matrix.at<double>(2, 0);
    camera_info.r[7] = calib_params.rectification_matrix.at<double>(2, 1);
    camera_info.r[8] = calib_params.rectification_matrix.at<double>(2, 2);

    camera_info.p.fill(0.0);
    camera_info.p[0] = calib_params.projection_matrix.at<double>(0, 0);
    camera_info.p[2] = calib_params.projection_matrix.at<double>(0, 2);
    camera_info.p[3] = calib_params.projection_matrix.at<double>(0, 3);
    camera_info.p[5] = calib_params.projection_matrix.at<double>(1, 1);
    camera_info.p[6] = calib_params.projection_matrix.at<double>(1, 2);
    camera_info.p[7] = calib_params.projection_matrix.at<double>(1, 3);
    camera_info.p[10] = 1.0;
    camera_info.p[11] = calib_params.projection_matrix.at<double>(2, 3);
    return camera_info;
}

ZedCpuRos2Wrapper::ZedCpuRos2Wrapper(
    const rclcpp::NodeOptions &options, int device_id)
    : Node("zed_cpu_ros2_wrapper", options),
      device_id_(device_id),
      canceled_(false) {
    auto resolution =
        this->declare_parameter<std::string>("resolution", "HD1080");
    zed_params_.setResolution(resolution);
    auto fps = std::to_string(this->declare_parameter<int>("fps", 15));
    zed_params_.setFps(fps);

    publish_rate_ = this->declare_parameter<double>("publish_rate", -1.0);
    if (std::abs(publish_rate_) < std::numeric_limits<double>::epsilon()) {
        RCLCPP_WARN(
            get_logger(),
            "Invalid publish_rate = 0. Use default value -1 instead");
        publish_rate_ = -1.0;
    }

    if (publish_rate_ > 0) {
        auto duration =
            std::chrono::nanoseconds(static_cast<int>(1e9 / publish_rate_));
        image_pub_timer_ = this->create_wall_timer(
            duration, [this]() { this->publish_next_frame_ = true; });
        publish_next_frame_ = false;
    } else {
        publish_next_frame_ = true;
    }
    use_camera_buffer_timestamps_ =
        declare_parameter<bool>("use_camera_buffer_timestamps", false);
    intra_process_comm_ = declare_parameter<bool>("intra_process_comm", false);
    compressed_ = declare_parameter<bool>("compressed", true);
    undistorted_ = declare_parameter<bool>("undistorted", true);

    auto left_image_topic_name = this->declare_parameter<std::string>(
        "left_image_topic_name", "zed/left_camera/image_raw");
    auto right_image_topic_name = this->declare_parameter<std::string>(
        "right_image_topic_name", "zed/right_camera/image_raw");
    left_camera_frame_id_ = this->declare_parameter<std::string>(
        "left_camera_frame_id", "left_camera");
    right_camera_frame_id_ = this->declare_parameter<std::string>(
        "right_camera_frame_id", "right_camera");

    auto use_sensor_data_qos =
        this->declare_parameter<bool>("use_sensor_data_qos", false);
    const auto qos =
        use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);

    // Publisher used for intra process comm
    left_camerainfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "zed/left_camera/camera_info", qos);
    right_camerainfo_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "zed/right_camera/camera_info", qos);
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "zed/left_camera/image_raw", qos);
    right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "zed/right_camera/image_raw", qos);
    left_compressed_image_pub_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "zed/left_camera/image_raw/compressed", qos);
    right_compressed_image_pub_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "zed/right_camera/image_raw/compressed", qos);

    // Publisher used for inter process comm
    left_camera_transport_pub_ = image_transport::create_camera_publisher(
        this, left_image_topic_name, qos.get_rmw_qos_profile());
    right_camera_transport_pub_ = image_transport::create_camera_publisher(
        this, right_image_topic_name, qos.get_rmw_qos_profile());

    left_camera_info_manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(
            (this), left_camera_frame_id_);
    right_camera_info_manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(
            (this), right_camera_frame_id_);

    this->openCamera();
    this->initCalibrationParms();
    this->cameraSpinner();
}

ZedCpuRos2Wrapper::~ZedCpuRos2Wrapper() {
    canceled_.store(true);
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
}

bool ZedCpuRos2Wrapper::openCamera() noexcept {
    sl_oc::video::VideoParams params;
    params.res = zed_params_.getResolution();
    params.fps = zed_params_.getFps();
    video_cap_ptr_ = std::make_shared<sl_oc::video::VideoCapture>(params);
    if (!video_cap_ptr_->initializeVideo(device_id_)) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera");
        return false;
    }

    serial_number_ = video_cap_ptr_->getSerialNumber();
    RCLCPP_INFO(
        get_logger(), "Opened camera with serial number: %d", serial_number_);

    return true;
}

bool ZedCpuRos2Wrapper::initCalibrationParms() {
    namespace fs = boost::filesystem;
    std::string home_path = getenv("HOME");
    std::string calib_dir_path =
        home_path + "/.config/zed-ros-wrapper/calibration";
    fs::path calib_dir_fs_path = fs::path(calib_dir_path);
    fs::create_directories(calib_dir_fs_path);
    fs::path calib_file_fs_path =
        calib_dir_fs_path /
        fs::path("SN" + std::to_string(serial_number_) + ".conf");
    if (!fs::exists(calib_file_fs_path)) {
        RCLCPP_INFO_STREAM(
            get_logger(), "[ZED ROS Wrapper] Downloading calibration file...");
        std::string url("'https://calib.stereolabs.com/?SN=");
        std::string cmd = "wget " + url + std::to_string(serial_number_) +
                          "' -O " + calib_file_fs_path.string();
        int res = system(cmd.c_str());
        if (res == EXIT_FAILURE) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "[ZED ROS Wrapper] Failed to download calibration file. "
                    << "Download it manually from following url. " << url
                    << serial_number_);
        }
    }
    int width, height;
    video_cap_ptr_->getFrameSize(width, height);
    cv::Mat camera_matrix_left, camera_matrix_right, distCoeffs_left,
        distCoeffs_right, R1, R2, P1, P2, map_left_x, map_left_y, map_right_x,
        map_right_y;
    sl_oc::tools::initCalibration(
        calib_file_fs_path.string(), cv::Size2i(width / 2, height),
        camera_matrix_left, camera_matrix_right, distCoeffs_left,
        distCoeffs_right, R1, R2, P1, P2, map_left_x, map_left_y, map_right_x,
        map_right_y);
#ifdef DEBUG
    std::cout << "Calibration parameters: " << std::endl;
    std::cout << " Camera Matrix L: \n"
              << camera_matrix_left << std::endl
              << std::endl;
    std::cout << " Camera Matrix R: \n"
              << camera_matrix_right << std::endl
              << std::endl;
    std::cout << " Distorsion L: \n"
              << distCoeffs_left << std::endl
              << std::endl;
    std::cout << " Distorsion R: \n"
              << distCoeffs_right << std::endl
              << std::endl;
    std::cout << " Rectification Matrix L: \n" << R1 << std::endl << std::endl;
    std::cout << " Rectification Matrix R: \n" << R2 << std::endl << std::endl;
    std::cout << " Projection Matrix L: \n" << P1 << std::endl << std::endl;
    std::cout << " Projection Matrix R: \n" << P2 << std::endl << std::endl;
#endif  // DEBUG

    left_calib_params_ = {
        static_cast<uint32_t>(width / 2),
        static_cast<uint32_t>(height),
        camera_matrix_left.at<double>(0, 0),
        camera_matrix_left.at<double>(1, 1),
        camera_matrix_left.at<double>(0, 2),
        camera_matrix_left.at<double>(1, 2),
        camera_matrix_left,
        distCoeffs_left,
        R1,
        P1,
        map_left_x,
        map_left_y};
    right_calib_params_ = {
        static_cast<uint32_t>(width / 2),
        static_cast<uint32_t>(height),
        camera_matrix_right.at<double>(0, 0),
        camera_matrix_right.at<double>(1, 1),
        camera_matrix_right.at<double>(0, 2),
        camera_matrix_right.at<double>(1, 2),
        camera_matrix_right,
        distCoeffs_right,
        R2,
        P2,
        map_right_x,
        map_right_y};

    auto left_camera_info_msg = CvtCalibParamsToCameraInfo(left_calib_params_);
    left_camera_info_manager_->setCameraInfo(left_camera_info_msg);
    auto right_camera_info_msg =
        CvtCalibParamsToCameraInfo(right_calib_params_);
    right_camera_info_manager_->setCameraInfo(right_camera_info_msg);

    return true;
}

void ZedCpuRos2Wrapper::cameraSpinner() {
    capture_thread_ = std::thread([this]() {
        uint64_t last_timestamp = 0;
        while (rclcpp::ok() && !canceled_.load()) {
            RCLCPP_DEBUG(get_logger(), "Capture thread running");
            sl_oc::video::Frame video_frame = video_cap_ptr_->getLastFrame(1);
            if (video_frame.data == nullptr) {
                RCLCPP_WARN(get_logger(), "Failed to get frame");
                continue;
            }
            if (last_timestamp == video_frame.timestamp) {
                continue;
            }
            if (publish_next_frame_ == false) {
                continue;
            }
            last_timestamp = video_frame.timestamp;

            rclcpp::Time timestamp;
            if (use_camera_buffer_timestamps_) {
                timestamp = rclcpp::Time(last_timestamp);
            } else {
                timestamp = this->get_clock()->now();
            }

            // cv::Mat yuv_image(
            //     video_frame.height, video_frame.width, CV_8UC2,
            //     video_frame.data);
            cv::Mat yuv_image(
                video_frame.height, video_frame.width, CV_8UC2);
            std::memcpy(yuv_image.data, video_frame.data, video_frame.width * video_frame.height * 2);
            cv::Mat bgr_image;
            cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_YUYV);

            std_msgs::msg::Header left_header, right_header;
            left_header.frame_id = left_camera_frame_id_;
            left_header.stamp = timestamp;

            right_header.frame_id = right_camera_frame_id_;
            right_header.stamp = timestamp;

            cv::Mat left_raw =
                bgr_image(cv::Rect(0, 0, bgr_image.cols / 2, bgr_image.rows));
            cv::Mat right_raw = bgr_image(cv::Rect(
                bgr_image.cols / 2, 0, bgr_image.cols / 2, bgr_image.rows));

            cv::Mat left_image_for_pub, right_image_for_pub;
            if (undistorted_) {
                cv::Mat left_rect, right_rect;
                try {
                    cv::remap(left_raw, left_rect,
                              left_calib_params_.map_x, left_calib_params_.map_y,
                              cv::INTER_LINEAR);
                    cv::remap(right_raw, right_rect,
                              right_calib_params_.map_x, right_calib_params_.map_y,
                              cv::INTER_LINEAR);
                } catch (cv::Exception &e) {
                    RCLCPP_WARN_STREAM_THROTTLE(
                        get_logger(), *this->get_clock(), 5000,
                        "[ZED ROS Wrapper] " << e.what());
                    continue;
                }
                left_image_for_pub = left_rect;
                right_image_for_pub = right_rect;
            } else {
                left_image_for_pub = left_raw;
                right_image_for_pub = right_raw;
            }

            auto left_camera_info_msg =
                std::make_unique<sensor_msgs::msg::CameraInfo>(
                    left_camera_info_manager_->getCameraInfo());
            left_camera_info_msg->header = left_header;
            auto right_camera_info_msg =
                std::make_unique<sensor_msgs::msg::CameraInfo>(
                    right_camera_info_manager_->getCameraInfo());
            right_camera_info_msg->header = right_header;

            if (compressed_) {
                auto left_image_msg =
                    cv_bridge::CvImage(left_header, "bgr8", left_image_for_pub)
                        .toCompressedImageMsg();
                auto right_image_msg =
                    cv_bridge::CvImage(right_header, "bgr8", right_image_for_pub)
                        .toCompressedImageMsg();
                left_compressed_image_pub_->publish(*left_image_msg);
                right_compressed_image_pub_->publish(*right_image_msg);
                left_camerainfo_pub_->publish(*left_camera_info_msg);
                right_camerainfo_pub_->publish(*right_camera_info_msg);
            } else if (intra_process_comm_) {
                auto left_image_msg =
                    cv_bridge::CvImage(left_header, "bgr8", left_image_for_pub)
                        .toImageMsg();
                auto right_image_msg =
                    cv_bridge::CvImage(right_header, "bgr8", right_image_for_pub)
                        .toImageMsg();
                left_image_pub_->publish(*left_image_msg);
                right_image_pub_->publish(*right_image_msg);
                left_camerainfo_pub_->publish(*left_camera_info_msg);
                right_camerainfo_pub_->publish(*right_camera_info_msg);
            } else {
                auto left_image_msg =
                    cv_bridge::CvImage(left_header, "bgr8", left_image_for_pub)
                        .toImageMsg();
                auto right_image_msg =
                    cv_bridge::CvImage(right_header, "bgr8", right_image_for_pub)
                        .toImageMsg();
                left_camera_transport_pub_.publish(
                    *left_image_msg, *left_camera_info_msg);
                right_camera_transport_pub_.publish(
                    *right_image_msg, *right_camera_info_msg);       
            }
            publish_next_frame_ = publish_rate_ < 0;
        }
    });
}
