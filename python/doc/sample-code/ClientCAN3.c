/*
 *  main.h
 *  MasterCAN
 *
 *  Created by Pablo Gutierrez on 7/22/17.
 *  Copyright 2017 E.S.O. All rights reserved.
 *
 */

////////////////////// NOTES ////////////////
/*
 You should use -l parameter, for listening on port 13370:
 
 $ nc -l 13370
 Now you have a tcp server on 127.0.0.1:13370
 
 On a second console you could connect to your server by using:
 
 $ nc 127.0.0.1 13370
 Please refer also to the official documentation link.
 */
//////////////////////////////////////////////

#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
#include "stdlib.h"		/// system("/bin/stty raw");
#include "stdbool.h"	/// bool
#include <unistd.h>
#include <stdint.h>

//#define ASCII_PROMPT_DEBUG

int		sock;

bool			flag_Connect;
bool			flag_TxLoop;
bool			flag_TxLoopi;

bool			flag_AgainLoop;

int				loopCount;

uint8_t			dummyDelay;
uint32_t		cuentaRx;

bool            flag_Z;        /// no message display ///

////////////////////////////////////////// PARSE FILE HERE ////////////////
/// ORIGINAL --> #include "Parse.h"
/*
 *  Parse.c
 *  MasterCAN
 *
 *  Created by Pablo Gutierrez on 7/22/17.
 *  Copyright 2017 E.S.O. All rights reserved.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#define STX 0x02
#define ETX 0x03
#define DLE 0x10

void print_frame(uint8_t *data, uint8_t len)
{
	if (len < 3) /// 
	{
		fprintf(stderr, "Invalid frame received: ");
		while (len--)
		{
			fprintf(stderr, "%02X ", *data++);
		}
		fprintf(stderr, "\n");
		return;
	}
	printf("%02hhx %04hx ", data[0], (((uint16_t)data[2]) << 8) | data[1]);
	len -= 3;
	data += 3;
    while (len--)
    {
        printf("%02X ", *data++);
    }
    printf("\n");
	//fflush(NULL);
}

void decode(uint8_t data)
{
    static struct
    {
        uint8_t buffer[16];
        uint8_t buflen;
        unsigned sync : 1;
        unsigned dle  : 1;
    } state = { .sync = 0, .dle = 0 };
	
    if (data == DLE && !state.dle)
    {
        state.dle = 1;
        return;
    }
	
    if (state.dle)
    {
        state.dle = 0;
        switch (data)
        {
            case STX:
                state.sync = 1;
                state.buflen = 0;
                return;
				
            case ETX:
                if (state.sync)
                {
                    state.sync = 0;
					cuentaRx++;
                    if (flag_Z) 
                        print_frame(state.buffer, state.buflen);
                    else {
						fflush(stderr);
                        fprintf(stderr,"%u\r", cuentaRx);
					}
					
                }
                return;
				
            case DLE:
                break;
				
            default:
                state.sync = 0;
                return;
        }
    }
	
    if (state.sync)
    {
        if (state.buflen < sizeof(state.buffer))
        {
            state.buffer[state.buflen++] = data;
        }
        else
        {
            int i;
            fprintf(stderr, "Ignoring frame, maximum length exceeded:");
            for (i=0; i<sizeof(state.buffer); i++)
            {
                fprintf(stderr, "%02X ", state.buffer[i]);
            }
            fprintf(stderr, "\n");
            state.sync = 0;
        }
    }
    else
    {
        state.sync = 0;
    }
    return;
}

int send_frame(uint8_t node, uint16_t id, uint8_t *data, uint8_t datalen)
{
    int i = 0;
    uint8_t buf[32];
	
#define byte_stuff(b) if ((b)==DLE) buf[i++] = (b); buf[i++] = (b);
    
	buf[i++] = DLE;
    buf[i++] = STX;
	
    byte_stuff(node);
    byte_stuff(id);
    byte_stuff(id >> 8);
    while (datalen--)
    {
        byte_stuff(*data);
        data++;
    }
    buf[i++] = DLE;
    buf[i++] = ETX;
	
    if (send(sock, buf, i, 0) < 0)
    {
        perror("send_f");
        return -1;
    }
	return 0;
}

int send_ascii(char *texto)
{
	size_t	length;
	
	length = strlen(texto);
	
	if (send(sock, texto, length, 0) < 0)
    {
        perror("send_a");
        return -1;
    }
	return 0;
}


int parse_line(const char *line)
{
    uint8_t node, data[8];
    uint16_t id;
    char extra[3] = "";
    int n = sscanf(line, "%2hhx %4hx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2s",
				   &node, &id, data, data+1, data+2, data+3,
				   data+4, data+5, data+6, data+7, extra);
    if (n < 2)
    {
        fprintf(stderr, "Syntax error: correct is \"bus id [data] ...\"\n");
        return 0;
    }
    n -= 2; // n is now the number of data bytes
	
    if (id > 0x7FF)
    {
        fprintf(stderr, "Error: invalid id - it must be less than 0x800\n");
        return 0;
    }
	
    if (strlen(extra) > 0)
    {
        fprintf(stderr, "Warning: extra input after 8th data byte ignored\n");
    }
	
    if (send_frame(node, id, data, n) < 0)
    {
        return -1;
    }
	
    return 0;
}
///////////////////////////////////////////////////////////////////////////

////////////////////////////////////////// TALK FILE HERE /////////////////
/// ORGINAL --> #include "Talk.h"
/*
 *  Talk.c
 *  MasterCAN
 *
 *  Created by Pablo Gutierrez on 7/22/17.
 *  Copyright 2017 E.S.O. All rights reserved.
 *
 */

#include <stdio.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
//#include <string.h>
#include <sys/types.h>

#include <netdb.h>
#include <sys/socket.h>

//#include <netinet/in.h>
#include <arpa/inet.h>		/// inet_addr //

#include <unistd.h>			/// read, select & close ///

#include <netinet/tcp.h>	/// TCP_NODELAY

int make_socket(const char *ip, uint16_t port)
{
    int sck;
    struct sockaddr_in addr;
	
	long value = 1;
	
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);
	
    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
        fprintf(stderr, "invalid ip address\n");
        return -1;
    }
	
    sck = socket (PF_INET, SOCK_STREAM, 0);
	
    if (sck < 0)
    {
        perror("socket");
        return -1;
    }
	
    if (connect(sck, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        perror("connect");
        close(sck);
        return -1;
    }
	
	setsockopt(sck, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value));
	
	return sck;
}


int recieve_check(void)
{
	int		i;
	int		nread;	
	char	buffer[0x40];
	fd_set	rfds;
	
	struct timeval timeout = { .tv_sec = 0,.tv_usec = 50000 };
	
	FD_ZERO(&rfds);
	
	FD_SET(sock, &rfds);
	
	if (select(sock + 1, &rfds, NULL, NULL, &timeout) < 0)
	{
		perror("select");
		return -1;
	}
	
	if (FD_ISSET(sock, &rfds))
	{
		nread = recv(sock, buffer, sizeof(buffer), 0);
		if (nread < 0)
		{
			perror("recv_ck");
			return -1;
		}
		else if (nread == 0)
		{
			fprintf(stderr, "client closed connection\n");
			return -1;
		}
		for (i = 0; i<nread; i++)
		{
#ifdef ASCII_PROMPT_DEBUG			
			printf("%c",buffer[i]); /// *** Debug *** ////
#endif
			//printf("%d\r\n",i); /// *** Debug *** ////
			decode(buffer[i]);
		}
	}
	return 0;
}
///////////////////////////////////////////////////////////////////////////
////////////////////// MAIN FILE HERE /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void testLoop(void)
{
	int		i;
#ifdef ASCII_PROMPT_DEBUG	
	char	texto[32];
#endif
	
	//	send_frame(0xFF, 0x0, NULL, 0); /// Check Status Before 
	
	loopCount = 0;
	for (i = 0; i < 20000; i++)
	{
		if (flag_Connect) {
			
			uint8_t data[8] = { i & 0xff, i >> 8, 2, 3, 4, 5, 6, 7 };
	
			
			///////////////////////////////////////////// TEST /////////////////////////////// 
			send_frame(0, 0x601, data, 8);
			send_frame(1, 0x602, data, 8);
			
			send_frame(2, 0x603, data, 8);
			send_frame(3, 0x604, data, 8);
			send_frame(4, 0x605, data, 8);
			
			send_frame(5, 0x606, data, 8);
			
			///////////////////////////////////////////// DELAY //////////////////////////////
			if (dummyDelay) {
				send_frame(6, 0x777, &dummyDelay, 1);
			} 
			
			////////////////////////////////////////////// DEBUG /////////////////////////////
			if (flag_TxLoopi) {
				flag_TxLoopi = false;
				printf("Loopi%d\n",i);
			}
			
#ifdef ASCII_PROMPT_DEBUG
			sprintf(texto,"Tx%d\r\n",i);
			send_ascii(texto);
#endif
		}
		if (flag_TxLoop == false) {
			break; /// exit ///
		}
	}
	loopCount = i;
	printf("LoopDone%d\n",loopCount);
	
	//	send_frame(0xFF, (i + 1), NULL, 0); /// Check Status After
}


void* threadTx(void *arg)
{	
	printf("Tx thread\n");
	while (1) 
	{
		if (flag_TxLoop) {
			testLoop();
			flag_TxLoop = false;
		}
    }
    return NULL;
}


void* threadRx(void *arg)
{	
	printf("Rx thread\n");
	while (1) 
	{
		if (flag_Connect) {
			if (recieve_check()) flag_Connect = false;
		}
	}
    return NULL;
}

/////////////////////////////////// SERIAL USER INTERFACE //////////////////////
void printHelp(void)
{
	printf("Press Key:\n");
	printf("d = Set Dummy Delay (ms)\n");
	printf("0 = Send Sync 0\n");
	printf("1 = Send Sync 1\n");
	printf("x = Configure Sync Params\n");
	printf("e = Get Errors\n");
	printf("t = Tx Message\n");
	printf("l = Loop Test (start/stop\n");
	printf("i = Print Loop i step\n");
	printf("r = Print Num Rx Messages\n");
}

void* threadKey(void *arg)
{	
	int			in_c;
	uint8_t		data[8];	
	
	printf("Key thread\n");
	while (1) 
	{
		in_c = getchar();	/// holds here wating fo a key to be pressed ///
		
		switch (in_c) {
			case 'd':
				if (dummyDelay) dummyDelay = 0; else dummyDelay = 10;
				printf("Dummy Delay %d\n",dummyDelay);
				send_frame(6, 0x777, &dummyDelay, 1);
				break;
			case '0':
				printf("Tx Sync 0\n");
				data[0] = 0;
				send_frame(7, 0x888, data, 1);
				break;
			case '1':
				printf("Tx Sync 1\n");
				data[0] = 1;
				send_frame(7, 0x999, data, 1);
				break;
			case 'x':
				printf("Config Sync\n");
				uint8_t data0[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
				uint8_t data1[8] = { 11, 12, 13, 14, 15, 16, 17, 18 };
				send_frame(8, 0x0123, data0, 8);
				send_frame(9, 0x0456, data1, 8);
				data[0] = (0x02);
				send_frame(10, 0, data, 1);
				data[0] = 0x03;
				send_frame(11, 0, data, 1);
				break;
			case 'e':
				printf("Get Error\n");
				send_frame(0xFF, 0, NULL, 0); /// Check Error
				break;
			case 't':
				printf("Tx Test\n");
				uint8_t dataT[8] = { 8, 7, 6, 5, 4, 3, 2, 1 };
				send_frame(1, 0x615, dataT, 8);
				//send_frame(2, 0x615, dataT, 8);
				//send_frame(3, 0x615, dataT, 8);
				//send_frame(4, 0x615, dataT, 8);
				break;
			case 'a':
				send_ascii("Hola\r\n");	
				break;
			case 'l':
				if (flag_TxLoop) flag_TxLoop = false; else flag_TxLoop = true;	/// toggle ///
				//flag_TxLoop = true;
				break;
			case '\004': // Ctrl-D
				flag_Connect = false;
				break;
			case '\003': // Ctrl-C
				flag_Connect = false;
				break;	
			case 'c':
				printf("loopCount=%d\r\n",loopCount);
				break;
			case 'i':	
				flag_TxLoopi = true;
				break;
			case 'r':
				printf("\r\ncuentaRx= %u\r\n",cuentaRx);
				cuentaRx = 0;
				break;
			case 'h':
				printHelp();
				break;
			case 'z':
				if (flag_Z) flag_Z = false; else flag_Z = true;    /// toggle ///
				printf("flag_Z= %d\r\n", flag_Z);
				break;
			case 'w':
				if (flag_AgainLoop) flag_AgainLoop = false; else flag_AgainLoop = true;    /// toggle ///
				printf("flag_AgainLoop= %d\r\n", flag_AgainLoop);
				break;
		}
	}
    return NULL;
}

int main (int argc, const char * argv[]) 
{
    int			err;
	
	pthread_t	threadID[3];
	
	flag_Z = false;
	
	printf("Hello, World!\n");
	
	printHelp();
	
	fprintf(stderr, "%d %s\n",argc,argv[0]);
	
	const char *ip_addX = "192.168.0.10";
		
	int port_num = 4700;
	
	flag_Connect = false;
	
	fprintf(stderr, "Connect to: <ip address> %s <port> %d\n", ip_addX, port_num);
	
	sock = make_socket(ip_addX, port_num);
	
    if (sock < 0)
    {
        return -1;
    }
	
	flag_Connect	= true;
	flag_TxLoop		= false;
	flag_AgainLoop	= false;
	
	printf("Connected!\n");
	
	/////////////// Create Threads ////////////////////////////////////
	
	err = pthread_create(&(threadID[0]), NULL, &threadRx, NULL);
	if (err != 0)  printf("\ncan't create thread :[%s]", strerror(err));
	
	err = pthread_create(&(threadID[1]), NULL, &threadTx, NULL);
	if (err != 0)  printf("\ncan't create thread :[%s]", strerror(err));
	
	err = pthread_create(&(threadID[2]), NULL, &threadKey, NULL);
	if (err != 0)  printf("\ncan't create thread :[%s]", strerror(err));
	
	//////////// Send Test Ascii Frame ////////////////////////////////
#ifdef ASCII_PROMPT_DEBUG
	send_ascii("Hola\r\n");	
#endif
	///////////////////////////////////////////////////////////////////
	
	system("/bin/stty raw");	/// for getchar -> keyboard input direct. not wait for "enter" ///
	
	while (1)
	{
		if (flag_Connect == false) break;
		if (flag_AgainLoop) {
			if (flag_TxLoop == false) {
				sleep(10);
				system("clear");
				printf("Loop Again!\n");
				flag_TxLoop = true;
			}
		}
	}
	
	pthread_cancel(threadID[0]);
	pthread_cancel(threadID[1]);
	pthread_cancel(threadID[2]);
	close(sock);
	printf("Disconnected!\n");
	
    return 0;
}

