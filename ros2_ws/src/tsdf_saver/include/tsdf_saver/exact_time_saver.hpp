/* =====================================================================
 * MIT License
 * 
 * Copyright (c) 2025 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tsdf_saver
{

    class ExactTimeSaver : public rclcpp::Node
    {
    public:
        explicit ExactTimeSaver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ExactTimeSaver() override;

    private:
        using Trigger      = std_srvs::srv::Trigger;
        using CloudMsg     = sensor_msgs::msg::PointCloud2;
        using TimePointSec = double;

        void cloudCallback(const CloudMsg::SharedPtr msg);
        void triggerSave(TimePointSec t);

        rclcpp::Client<Trigger>::SharedPtr client_;
        rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;

        double save_time_threshold_{0.0};  // ROS2 param
        bool saved_{false};
    };

} // namespace tsdf_saver
