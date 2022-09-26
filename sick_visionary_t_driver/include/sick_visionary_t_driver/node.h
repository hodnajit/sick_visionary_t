#include <chrono>
#include <boost/thread.hpp>

#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "sick_visionary_t_driver/driver.h"

class DriverNode : public rclcpp::Node {
public:
    DriverNode();

private:
    void publish_frame(const Driver_3DCS::Data &data);

    void thr_publish_frame();

    void on_frame(const boost::shared_ptr<Driver_3DCS::Data> &data);

    void on_new_subscriber();

    void timer_callback();

/// Flag whether invalid points are to be replaced by NaN
    const bool SUPPRESS_INVALID_POINTS = true;

/// Distance code for data outside the TOF range
    const uint16_t NARE_DISTANCE_VALUE = 0xffffU;

    image_transport::Publisher g_pub_depth, g_pub_confidence, g_pub_intensity;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr g_pub_camera_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_points;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr g_pub_ios;

    Driver_3DCS::Control *g_control = nullptr;

    std::string g_frame_id;

/// If true: prevents skipping of frames and publish everything, otherwise use newest data to publish to ROS world
    bool g_publish_all;

    boost::mutex g_mtx_data;
    boost::shared_ptr <Driver_3DCS::Data> g_data;

    rclcpp::TimerBase::SharedPtr timer_;

};
