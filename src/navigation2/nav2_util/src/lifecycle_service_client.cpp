// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/lifecycle_service_client.hpp"

#include <string>
#include <chrono>
#include <memory>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using nav2::generate_internal_node;
using std::chrono::milliseconds;
using std::make_shared;
using std::string;
using namespace std::chrono_literals;

namespace nav2_util
{

LifecycleServiceClient::LifecycleServiceClient(const std::string & lifecycle_node_name)
: LifecycleServiceClient(lifecycle_node_name, generate_internal_node("lifecycle_service_client"))
{
}

LifecycleServiceClient::LifecycleServiceClient(
  const std::string & lifecycle_node_name,
  rclcpp::Node::SharedPtr parent_node)
: node_(parent_node),
  change_state_(lifecycle_node_name + "/change_state", parent_node,
    true /*creates and spins an internal executor*/),
  get_state_(lifecycle_node_name + "/get_state", parent_node,
    true /*creates and spins an internal executor*/)
{
  // Block until server is up
  rclcpp::Rate r(20);
  int wait_count = 0;
  while (!get_state_.wait_for_service(2s)) {
    // Only log every 10 attempts (every ~1 second) to reduce log spam
    if (wait_count % 10 == 0) {
      RCLCPP_INFO(
        parent_node->get_logger(),
        "Waiting for service %s...", get_state_.getServiceName().c_str());
    }
    wait_count++;
    r.sleep();
  }
}

bool LifecycleServiceClient::change_state(
  const uint8_t transition,
  const std::chrono::seconds timeout)
{
  if (!change_state_.wait_for_service(std::chrono::seconds(10))) {
    throw std::runtime_error("change_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  if (timeout.count() > 0) {
    auto response = change_state_.invoke(request, timeout);
    return response.get();
  } else {
    auto response = std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
    return change_state_.invoke(request, response);
  }
}

bool LifecycleServiceClient::change_state(std::uint8_t transition)
{
  return change_state(transition, std::chrono::seconds(-1));
}

uint8_t LifecycleServiceClient::get_state(
  const std::chrono::seconds timeout)
{
  if (!get_state_.wait_for_service(timeout)) {
    throw std::runtime_error("get_state service is not available!");
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

}  // namespace nav2_util
