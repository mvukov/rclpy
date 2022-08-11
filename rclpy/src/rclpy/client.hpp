// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLPY__CLIENT_HPP_
#define RCLPY__CLIENT_HPP_

#include <pybind11/pybind11.h>

#include <rcl/client.h>
#include <rmw/types.h>

#include <memory>

#include "clock.hpp"
#include "destroyable.hpp"
#include "node.hpp"

namespace py = pybind11;

namespace rclpy
{
class Client : public Destroyable, public std::enable_shared_from_this<Client>
{
public:
  /// Create a client
  /**
   * This function will create a client for the given service name.
   * This client will use the typesupport defined in the service module
   * provided as pysrv_type to send messages.
   *
   * Raises ValueError if the capsules are not the correct types
   * Raises RuntimeError if the client could not be created
   *
   * \param[in] node Node to add the client to
   * \param[in] pysrv_type Service module associated with the client
   * \param[in] service_name The service name
   * \param[in] pyqos QoSProfile python object for this client
   * \param[in] clock Clock to use for service event timestamps
   */
  Client(Node & node, py::object pysrv_type, const char * service_name, py::object pyqos,
      Clock & clock)
    : Client(node, pysrv_type, service_name, pyqos, rcl_publisher_get_default_options().qos,
        clock){};

  /// Create a client
  /**
   * This function will create a client for the given service name.
   * This client will use the typesupport defined in the service module
   * provided as pysrv_type to send messages.
   *
   * Raises ValueError if the capsules are not the correct types
   * Raises RuntimeError if the client could not be created
   *
   * \param[in] node Node to add the client to
   * \param[in] pysrv_type Service module associated with the client
   * \param[in] service_name The service name
   * \param[in] pyqos QoSProfile python object for this client
   * \param[in] pyqos_service_event_pub QoSProfile python object for the service event publisher
   * \param[in] clock Clock to use for service event timestamps
   */
  Client(Node & node, py::object pysrv_type, const char * service_name, py::object pyqos,
      py::object pyqos_service_event_pub, Clock & clock);

  ~Client() = default;

  /// Publish a request message
  /**
   * Raises ValueError if pyclient is not a client capsule
   * Raises RuntimeError if the request could not be sent
   *
   * \param[in] pyrequest request message to send
   * \return sequence_number Index of the sent request
   */
  int64_t
  send_request(py::object pyrequest);

  /// Check if a service server is available
  /**
   * Raises ValueError if the arguments are not capsules
   *
   * \return True if the service server is available
   */
  bool
  service_server_is_available();

  /// Take a response from a given client
  /**
   * Raises ValueError if pyclient is not a client capsule
   *
   * \param[in] pyresponse_type Instance of the message type to take
   * \return 2-tuple sequence number and received response, or None if there is no response
   */
  py::tuple
  take_response(py::object pyresponse_type);

  /// Get rcl_client_t pointer
  rcl_client_t *
  rcl_ptr() const
  {
    return rcl_client_.get();
  }

  /// Force an early destruction of this object
  void
  destroy() override;

private:
  Client(Node & node, py::object pysrv_type, const char * service_name, py::object pyqos,
      rmw_qos_profile_t pyqos_service_event_pub, Clock & clock);
  Node node_;
  std::shared_ptr<rcl_client_t> rcl_client_;
};

/// Define a pybind11 wrapper for an rclpy::Client
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void
define_client(py::object module);
}  // namespace rclpy

#endif  // RCLPY__CLIENT_HPP_
