#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


using std::placeholders::_1;

struct Point32 
{
  float x;
  float y;
  float z;
};

class ScanToPclNode : public rclcpp::Node 
{
public:
  ScanToPclNode()
  :Node("scan_to_pcl")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ScanToPclNode::scan_callback, this, _1));
  
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "scan_cloud", 10
    );
  }

private:

  bool convert(
    const sensor_msgs::msg::LaserScan& scan, //in
    sensor_msgs::msg::PointCloud2& cloud) const // out
  {
    cloud.header = scan.header;

    cloud.data.resize(0);
    size_t valid_points = 0;

    for(size_t i=0; i<scan.ranges.size(); i++)
    {
      // polar
      float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      float range = scan.ranges[i];

      if(range < scan.range_max && range > scan.range_min)
      {
        Point32 p;
        p.x = cos(angle) * range;
        p.y = sin(angle) * range;
        p.z = 0.0;

        // push back one byte after another
        Point32* p_ptr = &p;
        uint8_t* p_bytes_ptr = reinterpret_cast<uint8_t*>(p_ptr);
        for(size_t j=0; j<sizeof(Point32); j++)
        {
          cloud.data.push_back(p_bytes_ptr[j]);
        }
        valid_points++;
      } 
    }

    cloud.width = valid_points;
    cloud.height = 1;
    cloud.point_step = sizeof(Point32);
    cloud.row_step = valid_points * sizeof(Point32);
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    cloud.fields.push_back(field_x);
    cloud.fields.push_back(field_y);
    cloud.fields.push_back(field_z);

    return true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Scan received. Convert to PCL");
    sensor_msgs::msg::PointCloud2 cloud;

    // convert
    convert(msg, cloud);  

    pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToPclNode>());
    rclcpp::shutdown();

    return 0;
}