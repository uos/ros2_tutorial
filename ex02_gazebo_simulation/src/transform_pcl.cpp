#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <Eigen/Dense>

using std::placeholders::_1;

struct Point32 
{
  float x;
  float y;
  float z;
};

class TransformPclNode : public rclcpp::Node 
{
public:
  TransformPclNode()
  :Node("transform_pcl")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "scan_cloud", 10, std::bind(&TransformPclNode::cloud_callback, this, _1));
  
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "scan_cloud_transformed", 10
    );

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // TODO: make ROS parameter from this
    frame_id_to_ = "base_footprint";
  }

private:

  sensor_msgs::msg::PointCloud2 doTransform(
    const sensor_msgs::msg::PointCloud2& cloud_in, 
    const geometry_msgs::msg::TransformStamped& T) const
  {

    // convert Transform to Eigen type
    Eigen::Isometry3f Teig = tf2::transformToEigen(T.transform).cast<float>();
    // Eigen::Isometry3f Te

    // most of the fields stay the same: copy everything
    sensor_msgs::msg::PointCloud2 cloud_out = cloud_in;

    const sensor_msgs::msg::PointField* field_x = nullptr;
    const sensor_msgs::msg::PointField* field_y = nullptr;
    const sensor_msgs::msg::PointField* field_z = nullptr;

    // search for x y z fields
    for(size_t i=0; i<cloud_in.fields.size(); i++)
    {
      // get point to field
      const sensor_msgs::msg::PointField* field = &cloud_in.fields[i];
      if(field->name == "x")
      {
        field_x = field;
      } else if(field->name == "y") {
        field_y = field;
      } else if(field->name == "z") {
        field_z = field;
      }
    }

    if(!field_x || !field_y || !field_z)
    {
      RCLCPP_ERROR(this->get_logger(), "COULD NOT FIND X Y or Z");
      throw std::runtime_error("COULD NOT FIND X Y or Z");
    }

    const uint8_t* src_ptr = &cloud_in.data[0];
    uint8_t* tgt_ptr = &cloud_out.data[0];

    for(size_t i=0; i<cloud_in.width * cloud_in.height; i++)
    {
      // byte index: bi
      size_t bi = i * cloud_in.point_step;
      // byte index of point fields
      size_t bix = bi + field_x->offset;
      size_t biy = bi + field_y->offset;
      size_t biz = bi + field_z->offset;

      // get a point from bytes
      const float* x_src = reinterpret_cast<const float*>(src_ptr + bix);
      const float* y_src = reinterpret_cast<const float*>(src_ptr + biy);
      const float* z_src = reinterpret_cast<const float*>(src_ptr + biz);

      // create eigen vector
      Eigen::Vector3f Pl;
      Pl.x() = *x_src;
      Pl.y() = *y_src;
      Pl.z() = *z_src;

      // transform single point from laser (l) to e.g. base (b)
      Eigen::Vector3f Pb = Teig * Pl;

      // pointer to target cloud
      float* x_tgt = reinterpret_cast<float*>(tgt_ptr + bix);
      float* y_tgt = reinterpret_cast<float*>(tgt_ptr + biy);
      float* z_tgt = reinterpret_cast<float*>(tgt_ptr + biz);

      // write to target cloud
      *x_tgt = Pb.x();
      *y_tgt = Pb.y();
      *z_tgt = Pb.z();
    }

    // set new coordinate system
    cloud_out.header.frame_id = T.header.frame_id;

    return cloud_out;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2& cloud_in) const
  {
    RCLCPP_INFO(this->get_logger(), "PCL received. Transform to PCL");
    // sensor_msgs::msg::PointCloud2 cloud_out;

    geometry_msgs::msg::TransformStamped T;

    try {
      T = tf_buffer_->lookupTransform(
        frame_id_to_, cloud_in.header.frame_id,
        cloud_in.header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        frame_id_to_.c_str(), cloud_in.header.frame_id.c_str(), ex.what());
      return;
    }

    // Transform
    // - from: T.child_frame_id
    // - to:   T.header.frame_id

    // transform all points
    sensor_msgs::msg::PointCloud2 cloud_out = doTransform(cloud_in, T);
    pub_->publish(cloud_out);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string frame_id_to_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPclNode>());
    rclcpp::shutdown();

    return 0;
}