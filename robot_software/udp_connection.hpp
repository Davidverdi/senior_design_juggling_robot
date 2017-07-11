/* udp_connection.hpp

   David Verdi
   Created 4/28/2017
   Modified 4/28/2017

   This is the header file for the udp_connection class.

   Credit for much of the code goes to:
   Beej's Guide to Network Programming
   http://beej.us/guide/bgnet/output/html/multipage/index.html
   Brian "Beej Jorgensen" Hall
   Version 3.0.21
   June 8, 2016

   I wrapped the code from Beej's guide into a C++ class. 
*/

#ifndef __UDP_CONN_HPP__
#define __UDP_CONN_HPP__

#include <pigpio.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <pthread.h>
#include <string>

#define MAXBUFLEN 1024

class udp_connection
{
public:
	// PUBLIC VARIABLES

	// Mutex to lock RX command variable
	pthread_mutex_t RX_lock;

	// Mutex to lock LX command variable
	pthread_mutex_t LX_lock;

	// Variable to hold the current left-hand x demanded point
	double LX_demand;

	// Variable to hold the current right hand x demanded point
	double RX_demand;

	// Port number to listen on
	std::string port_no;


	// Variable to tell if meaningful data has come through from the kinect
	bool track_flag;

	// Variable to tell if listener is active
	bool listener_flag;

	// Current thread id of listener
	pthread_t *listener;

	// Socket fd - resource handle
	int sockfd;

	// Storage structure for sender IP
	struct sockaddr_storage their_addr;

	// Buffer to store message
	char buf[MAXBUFLEN];

	//Random stuff used by functions
	int numbytes;
	socklen_t addr_len;



	// PUBLIC FUNCTIONS

	// Default Constructor
	udp_connection();

	// Constructor
	udp_connection(std::string portNumber);

	// Destructor
	~udp_connection();

	// Command to start connection
	int start_listening();

	// Command to get LX tracking point
	double get_LX();

	// Command to get RX tracking point
	double get_RX();

	// Command to set LX tracking point
	void set_LX(double lxCommand);

	// Command to set RX tracking point
	void set_RX(double rxCommand);

    // Kills the child thread and releases resouces.
    // Basically a destructor.
	void kill_connection();
};

void* start_listener(void *data);

#endif
