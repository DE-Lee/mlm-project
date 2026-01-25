/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanToScanFilterChain
{
protected:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;

public:
  ScanToScanFilterChain(rclcpp::Node::SharedPtr node)
      : nh_(node)
  {
    // Subscribe to the raw scan topic
    scan_sub_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ScanToScanFilterChain::callback, this, std::placeholders::_1));

    // Advertise the filtered scan topic
    output_pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);
  }

  // Callback function for the raw scan data
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Publish the raw scan data directly
    output_pub_->publish(*msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("scan_to_scan_filter_chain");
  ScanToScanFilterChain t(nh);

  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}

