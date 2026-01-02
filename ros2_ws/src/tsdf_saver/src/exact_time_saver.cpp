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
#include "tsdf_saver/exact_time_saver.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tsdf_saver
{

    ExactTimeSaver::ExactTimeSaver(const rclcpp::NodeOptions& options)
    : Node("exact_time_saver", options)
    {
        // Parameter for triggering save
        this->declare_parameter<double>("save_time_threshold", 120.0);
        this->get_parameter("save_time_threshold", save_time_threshold_);

        RCLCPP_INFO(
            this->get_logger(),
            "ExactTimeSaver will trigger when cloud header time >= %.2f sec.",
            save_time_threshold_
        );

        this->client_ = this->create_client<Trigger>("/save_grid_mesh");

        this->cloud_sub_ = this->create_subscription<CloudMsg>(
            "/cloud_in",
            rclcpp::QoS(5).best_effort(),
            [this](CloudMsg::SharedPtr msg) { cloudCallback(std::move(msg)); }
        );
    }

    ExactTimeSaver::~ExactTimeSaver()
    {
        RCLCPP_INFO(this->get_logger(), "ExactTimeSaver shutting down.");
    }

    void ExactTimeSaver::cloudCallback(const CloudMsg::SharedPtr msg)
    {
        double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if (!this->saved_ && t >= this->save_time_threshold_) {
            saved_ = true;
            this->triggerSave(t);
        }
    }

    void ExactTimeSaver::triggerSave(TimePointSec t)
    {
        if (!this->client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN(this->get_logger(), "Service /save_grid_mesh unavailable.");
            return;
        }

        auto request = std::make_shared<Trigger::Request>();

        this->client_->async_send_request(
            request,
            [this, t](rclcpp::Client<Trigger>::SharedFuture future) 
            {
                auto resp = future.get();
                RCLCPP_INFO(
                    this->get_logger(),
                    "TSDF Save triggered at cloud time %.2f sec. Response: %s",
                    t,
                    resp->message.c_str()
                );
            }
        );
    }

} // namespace tsdf_saver

RCLCPP_COMPONENTS_REGISTER_NODE(tsdf_saver::ExactTimeSaver)
