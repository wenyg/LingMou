#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <chrono>
#include <functional>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include "rclcpp/rclcpp.hpp"
#include "google/protobuf/any.pb.h"
#include "xviz/v2/session.pb.h"
#include "xviz/v2/core.pb.h"
#include "xviz/v2/primitives.pb.h"
#include "xviz/v2/envelope.pb.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// PBE1 magic number
const uint8_t PBE1_MAGIC[4] = {0x50, 0x42, 0x45, 0x31};

constexpr double TWO_PI = 6.28318530717958647692;

std::string serializeEnvelope(const std::string& type, const google::protobuf::Message& message) {
  xviz::v2::Envelope envelope;
  envelope.set_type(type);
  envelope.mutable_data()->PackFrom(message);

  std::string pb_data;
  envelope.SerializeToString(&pb_data);

  std::string result;
  result.append(reinterpret_cast<const char*>(PBE1_MAGIC), 4);
  result.append(pb_data);

  return result;
}

std::string createXVIZMetadata() {
  xviz::v2::Metadata metadata;
  metadata.set_version("2.0.0");
  
  auto* log_info = metadata.mutable_log_info();
  log_info->set_start_time(0.0);
  log_info->set_end_time(3600.0);
  
  // Vehicle pose stream
  auto& streams = *metadata.mutable_streams();
  streams["/vehicle_pose"].set_category(xviz::v2::StreamMetadata::POSE);
  
  // Time series streams
  auto& speed_stream = streams["/vehicle/control/speed"];
  speed_stream.set_category(xviz::v2::StreamMetadata::TIME_SERIES);
  speed_stream.set_scalar_type(xviz::v2::StreamMetadata::FLOAT);
  speed_stream.set_units("m/s");
  
  auto& steering_stream = streams["/vehicle/control/steering"];
  steering_stream.set_category(xviz::v2::StreamMetadata::TIME_SERIES);
  steering_stream.set_scalar_type(xviz::v2::StreamMetadata::FLOAT);
  steering_stream.set_units("degrees");
  
  auto& gear_stream = streams["/vehicle/control/gear"];
  gear_stream.set_category(xviz::v2::StreamMetadata::TIME_SERIES);
  gear_stream.set_scalar_type(xviz::v2::StreamMetadata::INT32);
  
  // Primitive streams
  auto& box_stream = streams["/visualization/box"];
  box_stream.set_category(xviz::v2::StreamMetadata::PRIMITIVE);
  box_stream.set_primitive_type(xviz::v2::StreamMetadata::POLYGON);
  box_stream.set_coordinate(xviz::v2::StreamMetadata::VEHICLE_RELATIVE);
  
  auto& box_array_stream = streams["/visualization/box_array"];
  box_array_stream.set_category(xviz::v2::StreamMetadata::PRIMITIVE);
  box_array_stream.set_primitive_type(xviz::v2::StreamMetadata::POLYGON);
  box_array_stream.set_coordinate(xviz::v2::StreamMetadata::VEHICLE_RELATIVE);
  
  auto& points_stream = streams["/visualization/points"];
  points_stream.set_category(xviz::v2::StreamMetadata::PRIMITIVE);
  points_stream.set_primitive_type(xviz::v2::StreamMetadata::POINT);
  points_stream.set_coordinate(xviz::v2::StreamMetadata::VEHICLE_RELATIVE);
  
  auto& line_stream = streams["/visualization/line"];
  line_stream.set_category(xviz::v2::StreamMetadata::PRIMITIVE);
  line_stream.set_primitive_type(xviz::v2::StreamMetadata::POLYLINE);
  line_stream.set_coordinate(xviz::v2::StreamMetadata::VEHICLE_RELATIVE);
  
  return serializeEnvelope("xviz/metadata", metadata);
}

void addBoxPolygon(xviz::v2::Polygon* polygon,
                   double center_x,
                   double center_y,
                   double base_z,
                   double length,
                   double width,
                   double yaw) {
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  double corners[4][2] = {
    {length / 2.0, width / 2.0},
    {length / 2.0, -width / 2.0},
    {-length / 2.0, -width / 2.0},
    {-length / 2.0, width / 2.0}
  };

  for (int i = 0; i < 4; ++i) {
    double x = center_x + corners[i][0] * cos_yaw - corners[i][1] * sin_yaw;
    double y = center_y + corners[i][0] * sin_yaw + corners[i][1] * cos_yaw;
    polygon->add_vertices(x);
    polygon->add_vertices(y);
    polygon->add_vertices(base_z);
  }
}

std::string createStaticXVIZStateUpdate(double timestamp) {
  xviz::v2::StateUpdate state_update;
  state_update.set_update_type(xviz::v2::StateUpdate::SNAPSHOT);

  auto* update = state_update.add_updates();
  update->set_timestamp(timestamp);

  // Pose stream: mimic empty localization scenario (dummy pose)
  auto& poses = *update->mutable_poses();
  auto* pose = &poses["/vehicle_pose"];
  pose->set_timestamp(timestamp);
  pose->add_orientation(0.0);
  pose->add_orientation(0.0);
  pose->add_orientation(0.0);
  pose->add_position(0.0);
  pose->add_position(0.0);
  pose->add_position(0.0);
  auto* map_origin = pose->mutable_map_origin();
  map_origin->set_latitude(0.0);
  map_origin->set_longitude(0.0);
  map_origin->set_altitude(0.0);

  // Primitive streams
  auto& primitives = *update->mutable_primitives();

  auto* ego_box = &primitives["/visualization/box"];
  auto* ego_polygon = ego_box->add_polygons();
  addBoxPolygon(ego_polygon, 0.0, 0.0, -1.0, 4.5, 2.0, std::sin(timestamp) * 0.25);

  auto* box_array_stream = &primitives["/visualization/box_array"];
  for (int i = 0; i < 3; ++i) {
    auto* obstacle = box_array_stream->add_polygons();
    double offset = static_cast<double>(i) * 6.0 - 6.0;
    addBoxPolygon(obstacle, offset, 6.0, -1.2, 2.0, 1.6, 0.0);
  }

  auto* points_stream = &primitives["/visualization/points"];
  auto* points = points_stream->add_points();
  for (int i = 0; i < 20; ++i) {
    double sample = static_cast<double>(i) / 19.0;
    double x = -10.0 + sample * 20.0;
    double y = std::sin(timestamp + sample * TWO_PI) * 2.0;
    double z = -1.0 + sample * 0.5;
    points->add_points(x);
    points->add_points(y);
    points->add_points(z);
  }

  auto* line_stream = &primitives["/visualization/line"];
  auto* polyline = line_stream->add_polylines();
  for (int i = 0; i <= 12; ++i) {
    double ratio = static_cast<double>(i) / 12.0;
    double x = ratio * 20.0 - 10.0;
    double y = std::cos(timestamp + ratio * TWO_PI) * 3.0;
    double z = -0.8;
    polyline->add_vertices(x);
    polyline->add_vertices(y);
    polyline->add_vertices(z);
  }

  // Time-series values follow smooth curves so the front-end has changing data
  auto* ts_speed = update->add_time_series();
  ts_speed->set_timestamp(timestamp);
  ts_speed->add_streams("/vehicle/control/speed");
  ts_speed->mutable_values()->add_doubles(12.5 + std::sin(timestamp * 0.5) * 2.0);

  auto* ts_steering = update->add_time_series();
  ts_steering->set_timestamp(timestamp);
  ts_steering->add_streams("/vehicle/control/steering");
  ts_steering->mutable_values()->add_doubles(std::sin(timestamp) * 20.0);

  auto* ts_gear = update->add_time_series();
  ts_gear->set_timestamp(timestamp);
  ts_gear->add_streams("/vehicle/control/gear");
  int gear = static_cast<int>(std::fmod(std::floor(timestamp), 5.0));
  ts_gear->mutable_values()->add_int32s(gear);

  return serializeEnvelope("xviz/state_update", state_update);
}

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
  explicit WebSocketSession(tcp::socket socket)
    : ws_(std::move(socket)), metadata_sent_(false) {}

  void run() {
    net::dispatch(
      ws_.get_executor(),
      beast::bind_front_handler(&WebSocketSession::onRun, shared_from_this()));
  }

  void send(std::string const& message) {
    net::post(
      ws_.get_executor(),
      beast::bind_front_handler(
        &WebSocketSession::onSend,
        shared_from_this(),
        message));
  }
  
  void sendMetadata() {
    if (!metadata_sent_) {
      send(createXVIZMetadata());
      metadata_sent_ = true;
    }
  }

  bool is_open() const {
    return ws_.is_open();
  }

private:
  void onRun() {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
      [](websocket::response_type& res) {
        res.set(beast::http::field::server, "xviz-converter-protobuf");
      }));

    ws_.async_accept(
      beast::bind_front_handler(&WebSocketSession::onAccept, shared_from_this()));
  }

  void onAccept(beast::error_code ec) {
    if (ec) {
      std::cerr << "Accept error: " << ec.message() << std::endl;
      return;
    }

    sendMetadata();
    doRead();
  }

  void doRead() {
    ws_.async_read(
      buffer_,
      beast::bind_front_handler(&WebSocketSession::onRead, shared_from_this()));
  }

  void onRead(beast::error_code ec, std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    if (ec == websocket::error::closed) {
      return;
    }

    if (ec) {
      std::cerr << "Read error: " << ec.message() << std::endl;
      return;
    }

    buffer_.clear();
    doRead();
  }

  void onSend(std::string message) {
    queue_.push_back(message);

    if (queue_.size() > 1) {
      return;
    }

    ws_.binary(true);
    ws_.async_write(
      net::buffer(queue_.front()),
      beast::bind_front_handler(&WebSocketSession::onWrite, shared_from_this()));
  }

  void onWrite(beast::error_code ec, std::size_t bytes_transferred) {
    boost::ignore_unused(bytes_transferred);

    if (ec) {
      std::cerr << "Write error: " << ec.message() << std::endl;
      return;
    }

    queue_.erase(queue_.begin());

    if (!queue_.empty()) {
      ws_.async_write(
        net::buffer(queue_.front()),
        beast::bind_front_handler(&WebSocketSession::onWrite, shared_from_this()));
    }
  }

  websocket::stream<beast::tcp_stream> ws_;
  beast::flat_buffer buffer_;
  std::vector<std::string> queue_;
  bool metadata_sent_;
};

class WebSocketListener : public std::enable_shared_from_this<WebSocketListener> {
public:
  WebSocketListener(net::io_context& ioc, tcp::endpoint endpoint)
    : ioc_(ioc), acceptor_(net::make_strand(ioc)), valid_(false) {
    beast::error_code ec;

    acceptor_.open(endpoint.protocol(), ec);
    if (ec) {
      std::cerr << "Open error: " << ec.message() << std::endl;
      return;
    }

    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
      std::cerr << "Set option error: " << ec.message() << std::endl;
      return;
    }

    acceptor_.bind(endpoint, ec);
    if (ec) {
      std::cerr << "Bind error: " << ec.message() << std::endl;
      return;
    }

    acceptor_.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
      std::cerr << "Listen error: " << ec.message() << std::endl;
      return;
    }
    
    valid_ = true;
  }

  void run() {
    if (valid_) {
      doAccept();
    }
  }
  
  bool is_valid() const {
    return valid_;
  }

  void broadcast(std::string const& message) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    auto it = sessions_.begin();
    while (it != sessions_.end()) {
      if ((*it)->is_open()) {
        (*it)->send(message);
        ++it;
      } else {
        it = sessions_.erase(it);
      }
    }
  }

private:
  void doAccept() {
    acceptor_.async_accept(
      net::make_strand(ioc_),
      beast::bind_front_handler(&WebSocketListener::onAccept, shared_from_this()));
  }

  void onAccept(beast::error_code ec, tcp::socket socket) {
    if (!ec) {
      auto session = std::make_shared<WebSocketSession>(std::move(socket));
      {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.push_back(session);
      }
      session->run();
    }

    doAccept();
  }

  net::io_context& ioc_;
  tcp::acceptor acceptor_;
  std::vector<std::shared_ptr<WebSocketSession>> sessions_;
  std::mutex sessions_mutex_;
  bool valid_;
};

class XvizConverterNode : public rclcpp::Node {
public:
  XvizConverterNode()
    : Node("xviz_converter_node"),
      ws_port_(8081),
      ioc_(),
      listener_(nullptr),
      init_success_(false),
      start_time_(std::chrono::steady_clock::now()) {

    auto const address = net::ip::make_address("0.0.0.0");
    auto const port = static_cast<unsigned short>(ws_port_);
    
    listener_ = std::make_shared<WebSocketListener>(ioc_, tcp::endpoint{address, port});
    
    if (!listener_->is_valid()) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Failed to start WebSocket server on port %d", ws_port_);
      return;
    }
    
    listener_->run();

    ioc_thread_ = std::thread([this]() {
      ioc_.run();
    });

    broadcast_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&XvizConverterNode::onBroadcastTimer, this));

    init_success_ = true;
    RCLCPP_INFO(this->get_logger(), 
                "xviz_converter_node started. XVIZ Protobuf WebSocket server on port %d", ws_port_);
  }

  ~XvizConverterNode() {
    ioc_.stop();
    if (ioc_thread_.joinable()) {
      ioc_thread_.join();
    }
  }
  
  bool is_initialized() const {
    return init_success_;
  }

private:
  void onBroadcastTimer() {
    if (!listener_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_).count();
    listener_->broadcast(createStaticXVIZStateUpdate(elapsed_seconds));
  }
  
  int ws_port_;
  net::io_context ioc_;
  std::shared_ptr<WebSocketListener> listener_;
  std::thread ioc_thread_;
  bool init_success_;
  std::chrono::steady_clock::time_point start_time_;
  rclcpp::TimerBase::SharedPtr broadcast_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XvizConverterNode>();
  
  if (!node->is_initialized()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                 "Failed to initialize xviz_converter_node");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
