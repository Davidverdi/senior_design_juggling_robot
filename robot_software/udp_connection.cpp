/* udp_connection.cpp

   David Verdi
   Created 4/28/2017
   Modified 4/28/2017

   This is a C++ class made to handle a UDP connection.

   Credit for much of this code goes to:
   Beej's Guide to Network Programming
   http://beej.us/guide/bgnet/output/html/multipage/index.html
   Brian "Beej Jorgensen" Hall
   Version 3.0.21
   June 8, 2016

   I wrapped the code from Beej's guide into a C++ class. 
*/

#include "udp_connection.hpp"

using namespace std;

// Default constructor:
udp_connection::udp_connection()
{
	track_flag = false;
	listener_flag = false;
	pthread_t* listener = NULL;
	sockfd = 0;

}

// Constructor
udp_connection::udp_connection(string portNumber)
{
    printf("UDP constructor.");
	track_flag = false;
	listener_flag = false;
	pthread_t* listener = NULL;
	sockfd = 0;
	port_no = portNumber;
	pthread_mutex_init(&LX_lock, NULL);
	pthread_mutex_init(&RX_lock, NULL);
}

// Destructor
udp_connection::~udp_connection()
{
	this->kill_connection();
	pthread_mutex_destroy(&LX_lock);
	pthread_mutex_destroy(&RX_lock);
}

void udp_connection::kill_connection()
{
	if(listener)
		{
			gpioStopThread(listener);
			listener = NULL;
		}

	if(sockfd)
	{
		close(sockfd);
		sockfd = 0;
	}
	track_flag = false;
	listener_flag = false;
}

int udp_connection::start_listening()
{
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, port_no.c_str(), &hints, &servinfo)) != 0) {
        printf("ERROR: getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("listener: socket");
            continue;
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("listener: bind");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "listener: failed to bind socket\n");
        return 2;
    }

    freeaddrinfo(servinfo);

    // Spawn a new thread to listen to the socket....
    listener = gpioStartThread(start_listener, this);
    return(0);
}

void udp_connection::set_LX(double lxCommand)
{
	track_flag = true;
	pthread_mutex_lock(&LX_lock);
	LX_demand = lxCommand;
	pthread_mutex_unlock(&LX_lock);
}

void udp_connection::set_RX(double rxCommand)
{
	track_flag = true;
	pthread_mutex_lock(&RX_lock);
	RX_demand = rxCommand;
	pthread_mutex_unlock(&RX_lock);
}

double udp_connection::get_RX()
{
	if(track_flag && listener_flag)
	{
		double output;
		pthread_mutex_lock(&RX_lock);
		output = RX_demand;
		pthread_mutex_unlock(&RX_lock);
		return(output);	
	}
	return(-1);
}

double udp_connection::get_LX()
{
	if(track_flag && listener_flag)
	{
		double output;
		pthread_mutex_lock(&LX_lock);
		output = LX_demand;
		pthread_mutex_unlock(&LX_lock);
		return(output);	
	}
	return(-1);
}

void *start_listener(void *data)
{
	// Cast pointer
	udp_connection *udp_ptr  = (udp_connection *) data;
	udp_ptr->listener_flag = true;
	printf("UDP Connection is up and Listening.");
	while(udp_ptr->listener_flag)
	{
		(udp_ptr->addr_len) = sizeof (udp_ptr->their_addr);

		// Listen to UDP port with recvfrom.
		// When we get something, process it and go.
		if (((udp_ptr->numbytes) = recvfrom((udp_ptr->sockfd), (udp_ptr->buf), MAXBUFLEN-1 , 0, (struct sockaddr *)&(udp_ptr->their_addr), &(udp_ptr->addr_len))) == -1) 
		{
			(udp_ptr->track_flag) = false;
			(udp_ptr->listener_flag) = false;
			printf("ERROR: recvfrom");
			return(NULL);
		}
		//printf("UDP message recd. %s \n",udp_ptr->buf); 

        // Packet is now saved in udp_ptr->buf
        // We can now parse it.
        // The string input is of the form:
        //"R0.125 L0.02" to command the right X to go to 0.125 and the left X to
        // go to 0.02. repeated calls to strtok returns segments of the string
        // split by the delimiter named in the first call. 
        char *split_string;
        split_string = strtok((udp_ptr->buf)," ");
        while(split_string != NULL)
        {
        	if (split_string[0] == 'L')
        	{
        		// We have a command for the left motor!
        		udp_ptr->set_LX(atof(split_string + 1));
			//printf("Left Motor command recd over UDP: %s \n", split_string);
        	}
        	if (split_string[0] == 'R')
        	{
        		// We have a command for the right motor!
        		udp_ptr->set_RX(atof(split_string + 1));
        	}
        	if (split_string[0] == 'X')
        	{
        		// Signal that transmission is over.
        		udp_ptr->listener_flag = false;
        		printf("Transmission is over on UDP. \n");
        		break;
        	}
        	split_string = strtok(NULL, " ");
        }
    }
    (udp_ptr->track_flag) = false;
    return(NULL);
}
