#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int main(int argc, char **argv) {
  const char* server_host = "localhost";
  int server_port = 9000;
  
  if (argc >= 2) {
    server_host = argv[1];
  }
  if (argc >= 3) {
    server_port = std::stoi(argv[2]);
  }

  std::cout << "Connecting to " << server_host << ":" << server_port << std::endl;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return 1;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port);
  
  if (inet_pton(AF_INET, server_host, &server_addr.sin_addr) <= 0) {
    std::cerr << "Invalid address: " << server_host << std::endl;
    close(sock);
    return 1;
  }

  if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Failed to connect to " << server_host << ":" << server_port << std::endl;
    close(sock);
    return 1;
  }

  std::cout << "Connected! Waiting for Control messages..." << std::endl;

  char buffer[4096];
  std::string line;
  
  while (true) {
    ssize_t n = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) {
      if (n == 0) {
        std::cout << "Connection closed by server" << std::endl;
      } else {
        std::cerr << "Receive error" << std::endl;
      }
      break;
    }

    buffer[n] = '\0';
    line += buffer;

    // Process complete lines (JSON Lines format)
    size_t pos;
    while ((pos = line.find('\n')) != std::string::npos) {
      std::string json_line = line.substr(0, pos);
      line = line.substr(pos + 1);
      
      if (!json_line.empty()) {
        std::cout << "Received Control: " << json_line << std::endl;
      }
    }
  }

  close(sock);
  return 0;
}

