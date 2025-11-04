#include <memory>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "demo_interfaces/msg/control.hpp"

class UiProxyNode : public rclcpp::Node {
public:
  UiProxyNode() : Node("ui_proxy_node"), tcp_port_(9000), server_fd_(-1) {
    subscription_ = this->create_subscription<demo_interfaces::msg::Control>(
      "/control_cmd", 10,
      std::bind(&UiProxyNode::onMsg, this, std::placeholders::_1));
    
    startTcpServer();
    RCLCPP_INFO(this->get_logger(), "ui_proxy_node started. Subscribed to /control_cmd, TCP server on port %d", tcp_port_);
  }

  ~UiProxyNode() {
    stopTcpServer();
  }

private:
  void startTcpServer() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
      return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(tcp_port_);

    if (bind(server_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind to port %d", tcp_port_);
      close(server_fd_);
      server_fd_ = -1;
      return;
    }

    if (listen(server_fd_, 5) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to listen");
      close(server_fd_);
      server_fd_ = -1;
      return;
    }

    // Start accept thread
    accept_thread_ = std::thread(&UiProxyNode::acceptClients, this);
  }

  void stopTcpServer() {
    if (server_fd_ >= 0) {
      close(server_fd_);
      server_fd_ = -1;
    }
    if (accept_thread_.joinable()) {
      accept_thread_.join();
    }
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      close(fd);
    }
    client_fds_.clear();
  }

  void acceptClients() {
    while (rclcpp::ok() && server_fd_ >= 0) {
      struct sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(server_fd_, (struct sockaddr *)&client_addr, &client_len);
      
      if (client_fd < 0) {
        if (server_fd_ >= 0) {
          RCLCPP_WARN(this->get_logger(), "Accept failed");
        }
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_fds_.push_back(client_fd);
      }
      
      RCLCPP_INFO(this->get_logger(), "Client connected from %s:%d (fd=%d)",
                  inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), client_fd);
    }
  }

  std::string msgToJson(const demo_interfaces::msg::Control & msg) {
    std::ostringstream oss;
    oss << "{"
        << "\"stamp\":{\"sec\":" << msg.stamp.sec << ",\"nanosec\":" << msg.stamp.nanosec << "},"
        << "\"speed_mps\":" << msg.speed_mps << ","
        << "\"steering_deg\":" << msg.steering_deg << ","
        << "\"gear\":" << msg.gear
        << "}\n";
    return oss.str();
  }

  void onMsg(const demo_interfaces::msg::Control::ConstSharedPtr msg) {
    std::ostringstream oss;
    oss << "Control: speed_mps=" << msg->speed_mps
        << ", steering_deg=" << msg->steering_deg
        << ", gear=" << msg->gear;
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    // Forward to TCP clients
    std::string json = msgToJson(*msg);
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = client_fds_.begin();
    while (it != client_fds_.end()) {
      int fd = *it;
      ssize_t sent = send(fd, json.c_str(), json.length(), MSG_NOSIGNAL);
      if (sent < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to send to client fd=%d, removing", fd);
        close(fd);
        it = client_fds_.erase(it);
      } else {
        ++it;
      }
    }
  }

  rclcpp::Subscription<demo_interfaces::msg::Control>::SharedPtr subscription_;
  int tcp_port_;
  int server_fd_;
  std::thread accept_thread_;
  std::vector<int> client_fds_;
  std::mutex clients_mutex_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UiProxyNode>());
  rclcpp::shutdown();
  return 0;
}

