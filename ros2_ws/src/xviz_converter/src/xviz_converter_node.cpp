#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <sstream>
#include <iostream>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/msg/control.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// WebSocket session for each connected client
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
  explicit WebSocketSession(tcp::socket socket)
    : ws_(std::move(socket)) {}

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

  bool is_open() const {
    return ws_.is_open();
  }

private:
  void onRun() {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
      [](websocket::response_type& res) {
        res.set(beast::http::field::server, "xviz-converter-websocket");
      }));

    ws_.async_accept(
      beast::bind_front_handler(&WebSocketSession::onAccept, shared_from_this()));
  }

  void onAccept(beast::error_code ec) {
    if (ec) {
      std::cerr << "Accept error: " << ec.message() << std::endl;
      return;
    }

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

    // Echo back (optional, can be removed if not needed)
    buffer_.clear();
    doRead();
  }

  void onSend(std::string message) {
    queue_.push_back(message);

    if (queue_.size() > 1) {
      return;
    }

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
};

// WebSocket server listener
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

// ROS2 node
class XvizConverterNode : public rclcpp::Node {
public:
  XvizConverterNode()
    : Node("xviz_converter_node"),
      ws_port_(8080),
      ioc_(),
      listener_(nullptr),
      init_success_(false) {
    
    // Create subscription to /control_cmd
    subscription_ = this->create_subscription<demo_interfaces::msg::Control>(
      "/control_cmd", 10,
      std::bind(&XvizConverterNode::onControlMsg, this, std::placeholders::_1));

    // Start WebSocket server
    auto const address = net::ip::make_address("0.0.0.0");
    auto const port = static_cast<unsigned short>(ws_port_);
    
    listener_ = std::make_shared<WebSocketListener>(ioc_, tcp::endpoint{address, port});
    
    // Check if listener was successfully initialized
    if (!listener_->is_valid()) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Failed to start WebSocket server on port %d", ws_port_);
      return;
    }
    
    listener_->run();

    // Run io_context in separate thread
    ioc_thread_ = std::thread([this]() {
      ioc_.run();
    });

    init_success_ = true;
    RCLCPP_INFO(this->get_logger(), 
                "xviz_converter_node started. WebSocket server on port %d", ws_port_);
  }
  
  bool is_initialized() const {
    return init_success_;
  }

  ~XvizConverterNode() {
    ioc_.stop();
    if (ioc_thread_.joinable()) {
      ioc_thread_.join();
    }
  }

private:
  std::string msgToJson(const demo_interfaces::msg::Control& msg) {
    std::ostringstream oss;
    oss << "{"
        << "\"timestamp\":{\"sec\":" << msg.stamp.sec 
        << ",\"nanosec\":" << msg.stamp.nanosec << "},"
        << "\"speed_mps\":" << msg.speed_mps << ","
        << "\"steering_deg\":" << msg.steering_deg << ","
        << "\"gear\":" << msg.gear
        << "}";
    return oss.str();
  }

  void onControlMsg(const demo_interfaces::msg::Control::ConstSharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "Received control: speed=%.2f, steering=%.2f, gear=%d",
                 msg->speed_mps, msg->steering_deg, msg->gear);

    // Convert to JSON and broadcast to all WebSocket clients
    std::string json = msgToJson(*msg);
    if (listener_) {
      listener_->broadcast(json);
    }
  }

  rclcpp::Subscription<demo_interfaces::msg::Control>::SharedPtr subscription_;
  int ws_port_;
  net::io_context ioc_;
  std::shared_ptr<WebSocketListener> listener_;
  std::thread ioc_thread_;
  bool init_success_;
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

