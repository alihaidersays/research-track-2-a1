// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2a1_ros2{

class MiniServer : public rclcpp::Node
{
public:
  MiniServer(const rclcpp::NodeOptions & options)
  : Node("mini_server", options)
  {
    service_ = this->create_service<AddTwoInts>(
      "add_two_ints", std::bind(&MiniServer::handle_service, this, _1, _2, _3));
  }

private:

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
  {
  (void)request_header;
  RCLCPP_INFO(
    this->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}
  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2a1_ros2::MiniServer) 

