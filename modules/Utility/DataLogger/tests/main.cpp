/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>
#include <iostream>

#include<stdio.h>
#include<stdlib.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<string.h>
#include <arpa/inet.h>
#include <fcntl.h>  // for open
#include <unistd.h>  // for close
#include<thread>
#include <mutex>

std::mutex mtx;

bool running = true;  // determines if the server is running or not

void error(const char *msg) {
    perror(msg);
    exit(1);
}

void serverEnd() {
  running = false;
}

void serverRun() {
  int sockfd, newsockfd, n;
  socklen_t clilen;
  char buffer[1500];
  const char msg[] = "HTTP/1.1 204 No Content";
  struct sockaddr_in serv_addr, cli_addr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);  // Create Socket

  if (sockfd < 0) error("ERROR opening socket");

  bzero(reinterpret_cast<char *>(&serv_addr), sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  serv_addr.sin_port = htons(8086);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) != 0) {  // Binding
     error("ERROR on binding");
     close(newsockfd);
     close(sockfd);
  }

  if (listen(sockfd, 10) != 0) error("ERROR on Listening");  // Listening

  clilen = sizeof(cli_addr);
  mtx.lock();
  while (running) {
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);  // Making new socket based on input to the server // NOLINT
    if (newsockfd < 0) error("ERROR on accept");


    bzero(buffer, 1500);

    n = read(newsockfd, buffer, sizeof(buffer));

    if (n <= 0) {
      error("ERROR reading from socket");
      break;
    }

    n = write(newsockfd, msg, sizeof(msg));

    if (n <= 0) {
      error("ERROR writing to socket");
      break;
    }

    const char* p = "\nWriting to Client complete.\n";

    send(newsockfd, p, strlen(p), 0);

    shutdown(newsockfd, SHUT_RDWR);

    if (close(newsockfd) != 0) {
    error("ERROR on closing");
    break;
    }
     sleep(3);
  }

  mtx.unlock();
  std::cout << "Server shutting down." << std::endl;
  shutdown(sockfd, SHUT_RDWR);
  close(sockfd);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  std::thread t1(serverRun);
  int res = RUN_ALL_TESTS();
  std::thread t2(serverEnd);
  t1.join();
  t2.join();

  return res;
}
