/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <manipulate_paper/movements.h>

int portno=8686;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[])
{
  ros::init (argc, argv, "move_server");
  int sockfd, newsockfd;
     socklen_t clilen;
     char buffer[256];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     int res =  bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr));
     if (res<0)
       error("Error on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");
     bzero(buffer,256);
     while(n = read(newsockfd,buffer,255)){
       if (n < 0) error("ERROR reading from socket");
       printf("Here is the message: %s\n",buffer);
       n = write(newsockfd,"I got your message",18);
       if (n < 0) error("ERROR writing to socket");
       //       find_table_edge();
     }
     close(newsockfd);
     close(sockfd);
     return 0; 
}
