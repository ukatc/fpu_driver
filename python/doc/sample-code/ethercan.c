#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/time.h>
#include <sys/types.h>
//#include <unistd.h>

#if defined(_WIN32)
#include <conio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <netdb.h>
#include <sys/socket.h>
#endif

#define STX 0x02
#define ETX 0x03
#define DLE 0x10

#define			MAX_THREADS	3
//#define			ASCII_PROMPT_DEBUG

//////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PARAMS ////////////////////////
/////////////////////////////////////////////////////////////////////////
BOOL			flag_Connect;
BOOL			flag_TxLoop;
BOOL			flag_TxLoopi;
BOOL			flag_AgainLoop;
BOOL			flag_ShowErr;

int				loopCount;

uint8_t			dummyDelay;
uint32_t		cuentaRx;

BOOL			flag_Z;		/// no message display ///
FILE			*log_file;
BOOL			flag_Verbose;

int sock;

#if defined(_WIN32)
static char * winsockstr(int error)
{
    switch (error)
    {
        case WSAEINTR          : return("INTR");
        case WSAEBADF          : return("BADF");
        case WSAEACCES         : return("ACCES");
        case WSAEFAULT         : return("FAULT");
        case WSAEINVAL         : return("INVAL");
        case WSAEMFILE         : return("MFILE");
        case WSAEWOULDBLOCK    : return("WOULDBLOCK");
        case WSAEINPROGRESS    : return("INPROGRESS");
        case WSAEALREADY       : return("ALREADY");
        case WSAENOTSOCK       : return("NOTSOCK");
        case WSAEDESTADDRREQ   : return("DESTADDRREQ");
        case WSAEMSGSIZE       : return("MSGSIZE");
        case WSAEPROTOTYPE     : return("PROTOTYPE");
        case WSAENOPROTOOPT    : return("NOPROTOOPT");
        case WSAEPROTONOSUPPORT: return("PROTONOSUPPORT");
        case WSAESOCKTNOSUPPORT: return("SOCKTNOSUPPORT");
        case WSAEOPNOTSUPP     : return("OPNOTSUPP");
        case WSAEPFNOSUPPORT   : return("PFNOSUPPORT");
        case WSAEAFNOSUPPORT   : return("AFNOSUPPORT");
        case WSAEADDRINUSE     : return("ADDRINUSE");
        case WSAEADDRNOTAVAIL  : return("ADDRNOTAVAIL");
        case WSAENETDOWN       : return("NETDOWN");
        case WSAENETUNREACH    : return("NETUNREACH");
        case WSAENETRESET      : return("NETRESET");
        case WSAECONNABORTED   : return("CONNABORTED");
        case WSAECONNRESET     : return("CONNRESET");
        case WSAENOBUFS        : return("NOBUFS");
        case WSAEISCONN        : return("ISCONN");
        case WSAENOTCONN       : return("NOTCONN");
        case WSAESHUTDOWN      : return("SHUTDOWN");
        case WSAETOOMANYREFS   : return("TOOMANYREFS");
        case WSAETIMEDOUT      : return("TIMEDOUT");
        case WSAECONNREFUSED   : return("WSAECONNREFUSED");
        case WSAELOOP          : return("LOOP");
        case WSAENAMETOOLONG   : return("NAMETOOLONG");
        case WSAEHOSTDOWN      : return("HOSTDOWN");
        case WSAEHOSTUNREACH   : return("HOSTUNREACH");
        case WSAENOTEMPTY      : return("NOTEMPTY");
        case WSAEPROCLIM       : return("PROCLIM");
        case WSAEUSERS         : return("USERS ");
        case WSAEDQUOT         : return("DQUOT");
        case WSAESTALE         : return("STALE");
        case WSAEREMOTE        : return("REMOTE ");
        case WSAEDISCON        : return("DISCON");
        case WSASYSNOTREADY    : return("SYSNOTREADY");
        case WSAVERNOTSUPPORTED: return("VERNOTSUPPORTED");
        case WSANOTINITIALISED : return("NOTINITIALISED");
        case WSAHOST_NOT_FOUND : return("HOST_NOT_FOUND");
        case WSATRY_AGAIN      : return("TRY_AGAIN");
        case WSANO_RECOVERY    : return("NO_RECOVERY");
        case WSANO_DATA        : return("NO_DATA");
        default : return("unknown socket error");
    }
}
#endif

#define MAX_CHAR_GUP		32

void save_frame(uint8_t *data, uint8_t len)
{
	char		texto[MAX_CHAR_GUP];

	if (len < 3) /// 
	{
		//fprintf(stderr, "Invalid frame received: ");
		fputs("Invalid frame received: ", log_file);
		while (len--)
		{
			//fprintf(stderr, "%02X ", *data++);
			sprintf_s(texto, MAX_CHAR_GUP, "%02X ", *data++);
			fputs(texto, log_file);
		}
		//fprintf(stderr, "\n");
		fputs("\n", log_file);
		return;
	}
	//printf("%02x %04x ", data[0], (((uint16_t)data[2]) << 8) | data[1]);
	sprintf_s(texto, MAX_CHAR_GUP, "%02x %04x ", data[0], ((((uint16_t)data[2]) << 8) | data[1]));
	fputs(texto, log_file);
	len -= 3;
	data += 3;
	while (len--)
	{
		//printf("%02X ", *data++);
		sprintf_s(texto, MAX_CHAR_GUP, "%02X ", *data++);
		fputs(texto, log_file);
	}
	//printf("\n");
	fputs("\n", log_file);
	fflush(NULL);
}

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
	printf("%02x %04x ", data[0], (((uint16_t)data[2]) << 8) | data[1]);
	len -= 3;
	data += 3;
    while (len--)
    {
        printf("%02X ", *data++);
    }
    printf("\n");
	fflush(NULL);
}

//////////////////////////////////////////////////////////////////////////
/////////// ETHERCAN INFO ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define MSG_TYPE_FVER		0xC0	/// Firmware Version
#define MSG_TYPE_MACA		0xC1	/// MAC Address
#define MSG_TYPE_EBUF		0xC2	/// Ethernet Buffer Sizes 16-bit values as rcv_use_max [w0] + rcv_free_now [w1] + snd_use_max [w2] + snd_free_now [w3]
#define MSG_TYPE_RXTX		0xC3	/// CANbus Rx Buffer Max | txBusy in bytes [5][4][3[2][1][0] + CANbus Tx delay in [7]6] = MSB [7][6][5][4][3][2][1][0] LSB
#define MSG_TYPE_ETHC		0xC4	/// Ethernet Counts: 32-bit RCV count : Ethernet -> EtherCAN -> CANbus
#define MSG_TYPE_GRUP		0xC5	/// Report Group
//////////////////////////////////////////////////////////////////////////
/////////// CANBUS INPUT /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define MSG_TYPE_CRX0		0xD0	/// CANbus 0 Input Count
#define MSG_TYPE_CRX1		0xD1	/// CANbus 1 Input Count
#define MSG_TYPE_CRX2		0xD2	/// CANbus 2 Input Count
#define MSG_TYPE_CRX3		0xD3	/// CANbus 3 Input Count
#define MSG_TYPE_CRX4		0xD4	/// CANbus 4 Input Count
#define MSG_TYPE_CRX5		0xD5	/// CANbus 5 Input Count
//////////////////////////////////////////////////////////////////////////
/////////// CANBUS OUTPUT ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define MSG_TYPE_CTX0		0xE0	/// CANbus 0 Output Count
#define MSG_TYPE_CTX1		0xE1	/// CANbus 1 Output Count
#define MSG_TYPE_CTX2		0xE2	/// CANbus 2 Output Count
#define MSG_TYPE_CTX3		0xE3	/// CANbus 3 Output Count
#define MSG_TYPE_CTX4		0xE4	/// CANbus 4 Output Count
#define MSG_TYPE_CTX5		0xE5	/// CANbus 5 Output Count

#define MSG_TYPE_ERRS		0xFF	/// EtherCAN Errors

struct msg_t
{
	union
	{
		uint32_t 	data[2];
		uint16_t 	data_word[4];
		uint8_t 	data_byte[8];
	};
	uint16_t 	cob_id;
	uint8_t 	bus;
	uint8_t 	data_length;
};


void print_message(struct msg_t *pmsg)
{
	int i;

	printf("%02X %04X ", pmsg->bus, pmsg->cob_id);

	for (i = 0; i<pmsg->data_length; i++)
	{
		printf("%02X ", pmsg->data_byte[i]);
	}
	printf("\n");
}

void print_Errors(uint8_t *data, uint8_t len)
{
	struct	msg_t	canMsg;
	uint8_t			i;
	uint16_t		error_word;
	uint16_t		mask_word;
	uint8_t			canStatus;

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
	canMsg.bus = data[0];
	canMsg.cob_id = ( (((uint16_t)data[2]) << 8) | data[1]);
	//printf("%02x %04x ", data[0], (((uint16_t)data[2]) << 8) | data[1]);
	len -= 3;
	data += 3;
	canMsg.data_length = len;
	i = 0;
	while (len--)
	{
		canMsg.data_byte[i++] = *data++;
		//printf("%02X ", *data++);
	}
	//printf("\n");
	switch (canMsg.bus)
	{
	case MSG_TYPE_FVER:
		printf("Firmware Version=  %u.%u\n", canMsg.data_byte[1], canMsg.data_byte[0]);
		flag_ShowErr = FALSE;
		break;
	case MSG_TYPE_EBUF:
		printf("RCV-buf use: max= %4u free= %4u (1072)", canMsg.data_word[0], canMsg.data_word[1]);
		printf("\n");
		printf("SND-buf use: max= %4u free= %4u (2680)", canMsg.data_word[2], canMsg.data_word[3]);
		break;
	case MSG_TYPE_RXTX:
		for (i = 0; i < 6; i++) {
			canStatus = canMsg.data_byte[i];
			printf("rxQueue%d Max  %2u (0..63)", i, (0x7F & canStatus));
			if (canStatus & 0x80) printf("  -tx Busy!");
			if (i<5) printf("\n");
		}
		printf("\nCanBusTx wait Max=%u (counts)", canMsg.data_word[3]);
		break;
	case MSG_TYPE_MACA:
		printf("MAC Address= ");
		for (i = 0; i < 6; i++) {
			printf("0x%02X", canMsg.data_byte[i]);
			if (i<5) printf(".");
		}
		printf("\n");
		flag_ShowErr = FALSE;
		break;
	case MSG_TYPE_ETHC:
		printf("RCV-Total messages %lu\n", canMsg.data[0]);
		printf("RCV-Total packets  %lu", canMsg.data[1]);
		break;
	case MSG_TYPE_CRX0:
	case MSG_TYPE_CRX1:
	case MSG_TYPE_CRX2:
	case MSG_TYPE_CRX3:
	case MSG_TYPE_CRX4:
	case MSG_TYPE_CRX5:
		printf("CAN-RX %d ", (canMsg.bus- MSG_TYPE_CRX0));
		printf("in  %8u err %8u ", canMsg.data[0], canMsg.data[1]);
		break;
	case MSG_TYPE_CTX0:
	case MSG_TYPE_CTX1:
	case MSG_TYPE_CTX2:
	case MSG_TYPE_CTX3:
	case MSG_TYPE_CTX4:
	case MSG_TYPE_CTX5:
		printf("CAN-TX %d ", (canMsg.bus- MSG_TYPE_CTX0));
		printf("out %8u err %8u ", canMsg.data[0], canMsg.data[1]);
		break;
	case MSG_TYPE_ERRS:	
		printf("ERROR WORD = ");
		error_word = canMsg.data_word[0];
		///error_word = 0x8000; /// *** to test *** ///
		printf("%04x ", canMsg.data_word[0]);
		if (error_word == 0x0000) {
			printf("NO Error");
		}
		else {
			printf("\n");
			mask_word = 0x0001;
			for (i = 0; i < 6; i++) {
				if (error_word & mask_word) printf("txCBus%d Error\n", i);
				mask_word <<= 1;
			}
			if (error_word & 0x0040) printf("Ethernet RCV Error\n");
			if (error_word & 0x0080) printf("bit7<>0 Error\n");
			mask_word = 0x0100;
			for (i = 0; i < 6; i++) {
				if (error_word & mask_word) printf("txCBus%d Error\n", i);
				mask_word <<= 1;
			}
			if (error_word & 0x4000) printf("Ethernet SND Error\n");
			if (error_word & 0x8000) printf("bit15<>0 Error\n");
		}
		flag_ShowErr = FALSE;
		break;
	default:
		break;
	}
	printf("\n");
	//print_message(&canMsg);
	fflush(NULL);
	
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
					///cuentaRx++;
					
					////////////// ERROR GROUP ////////////////////////////
					if (flag_ShowErr) {
						print_Errors(state.buffer, state.buflen);
					}
					else {
						/////////////////////////////////////////////////////////
						cuentaRx++;
						//////////////// SAVE LOG ////////////////////////////////
						if (log_file) save_frame(state.buffer, state.buflen);
						///////////////// PRINT TERMINAL ///////////////////////
						if (flag_Z)
							print_frame(state.buffer, state.buflen);
						else
							printf("%u\r", cuentaRx);
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

/*
static void byte_stuff(uint8_t buf[], size_t *offset, uint8_t data)
{
	if (data == DLE)
	{
		buf[(*offset)++] = DLE;
	}
	buf[(*offset)++] = data;
}
*/

int send_frame(uint8_t bus, uint16_t cob_id, uint8_t *data, uint8_t datalen)
{
    int i = 0;
	int j;
    uint8_t buf[32];
#define byte_stuff(b) if ((b)==DLE) buf[i++] = (b); buf[i++] = (b);
    buf[i++] = DLE;
    buf[i++] = STX;

	byte_stuff(bus);
	byte_stuff((uint8_t)cob_id);	/// byte_stuff(cob_id); *** NEEDS TO BE TYPE CASTED *** ///
	byte_stuff(cob_id >> 8);
    while (datalen--)
    {
		//byte_stuff(buf, &i, *data);			
		byte_stuff(*data);
        data++;
    }
    buf[i++] = DLE;
    buf[i++] = ETX;
	
	if (flag_Verbose) {
		for (j = 0; j < i; j++) printf("%02x ", buf[j]);
	}

    if (send(sock, buf, i, 0) < 0)
    {
#if defined(_WIN32)
        int err = WSAGetLastError();
        fprintf(stderr, "send: %s\n", winsockstr(err));
#else
        perror("send");
#endif
        return -1;
    }
	return 0;
}


int parse_line(const char *line)
{
	uint8_t		type;
	uint8_t		data[8];
    uint16_t	id;
    char		extra[3] = "";
    int n = sscanf(line, "%2hhx %4hx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2hhx %2s",
            &type, &id, data, data+1, data+2, data+3,
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

    if (send_frame(type, id, data, n) < 0)
    {
        return -1;
    }

    return 0;
}

int make_socket(const char *ip, uint16_t port)
{
    int sck;
    struct sockaddr_in addr;
    struct hostent *hostinfo;
#if defined(_WIN32)
    WORD wVersionRequested;
    WSADATA wsaData;
	DWORD value = 1;

    wVersionRequested = MAKEWORD(2, 2);

    if (WSAStartup(wVersionRequested, &wsaData) != 0 ||
            LOBYTE( wsaData.wVersion ) != 2 ||
            HIBYTE( wsaData.wVersion ) != 2 )
    {
        WSACleanup();
        fprintf(stderr, "Failed to initialize Windows Sockets\n");
        return -1;
    }
#endif

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
#if defined(_WIN32)
        int err = WSAGetLastError();
        fprintf(stderr, "socket: %s\n", winsockstr(err));
        WSACleanup();
#else
        perror("socket");
#endif
        return -1;
    }

    if (connect(sck, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
#if defined(_WIN32)
        int err = WSAGetLastError();
        fprintf(stderr, "connect: %s\n", winsockstr(err));
        closesocket(sck);
        WSACleanup();
#else
        perror("connect");
        close(sck);
#endif
        return -1;
    }
 
	//setsockopt(sck, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value));

	return sck;
}

int recieve_check(void)
{
	char line[0x100];
	int linep = 0;
	int i, nread, n;
	char buffer[0x40];
	fd_set rfds;
	struct timeval timeout = { .tv_sec = 0,.tv_usec = 50000 };

	FD_ZERO(&rfds);
	FD_SET(sock, &rfds);

	if (select(sock + 1, &rfds, NULL, NULL, &timeout) < 0)
	{
		int err = WSAGetLastError();
		fprintf(stderr, "select: %s\n", winsockstr(err));

		return -1;
	}

	if (FD_ISSET(sock, &rfds))
	{
		nread = recv(sock, buffer, sizeof(buffer), 0);
		if (nread < 0)
		{
			int err = WSAGetLastError();
			fprintf(stderr, "recv: %s\n", winsockstr(err));
		}
		else if (nread == 0)
		{
			fprintf(stderr, "client closed connection\n");
			return 0;
		}
		for (i = 0; i<nread; i++)
		{
			if (flag_Verbose) printf("%02x ", buffer[i]);
			decode(buffer[i]);
		}
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////
/////////////////////////////// TEST LOOPS /////////////////////////////
///////////////////////////////////////////////////////////////////////
#define TEST_UNO
//#define TEST_DOS
//#define TEST_TRES

void testLoop(void)
{
	int		i,j;

#ifdef ASCII_PROMPT_DEBUG	
	char	texto[32];
#endif

	//	send_frame(0xFF, 0x0, NULL, 0); /// Check Status Before 

	loopCount = 0;

#ifdef TEST_UNO
	for (i = 0; i < 20000; i++)
	{
		if (flag_Connect) {
			/////////////////////////////////////// DATA COUNT /////////////////////////////////
			//uint8_t data[8] = { i & 0xff, i >> 8, 2, 3, 4, 5, 6, 7 };
			////////////////////////////////////////// PING //////////////////////////////////////////////////////////
			uint8_t data[8] = { 7,i & 0xff, i >> 8, 2, 3, 4, 5, 6, }; // 7 = PING , rest for secuencial order check //
			///////////////////////////////////////////// TEST ///////////////////////////////////////////////////////
			send_frame(0, 0x601, data, 8);
			//send_frame(1, 0x602, data, 8);
			//send_frame(1, 0x610, data, 8);
			//send_frame(1, 0x603, data, 8);
			//send_frame(1, 0x604, data, 8);
			//send_frame(2, 0x603, data, 8);
			//send_frame(3, 0x604, data, 8);
			//send_frame(4, 0x605, data, 8);
			//send_frame(5, 0x606, data, 8);
#endif
#ifdef TEST_DOS
		for (i = 0; i < 1000; i++)
		{
			if (flag_Connect) {
				////////////////////////////////////////// PING //////////////////////////////////////////////////////////
				uint8_t data[8] = { 7,i & 0xff, i >> 8, 2, 3, 4, 5, 6, }; // 7 = PING , rest for secuencial order check //
				///////////////////////////////////////////// TEST ///////////////////////////////////////////////////////
				for (j = 0; j < 38; j++) {
					send_frame(5, 0x601 + j, data, 8);
				}
#endif
			///////////////////////////////////////////// DELAY //////////////////////////////
			if (dummyDelay) {
				send_frame(6, 0x777, &dummyDelay,1);
			}

			////////////////////////////////////////////// DEBUG /////////////////////////////
			if (flag_TxLoopi) {
				flag_TxLoopi = FALSE;
				printf("Loop Sent = %d\n", (i+1));
			}

#ifdef ASCII_PROMPT_DEBUG
			sprintf(texto, "Tx%d\r\n", i);
			send_ascii(texto);
#endif
		}
		if (flag_TxLoop == FALSE) {
			i++; /// so that ir represents what was sent 
			break; /// exit ///
		}
	}
	loopCount = i;
	printf("LoopDone%d\n", loopCount);	/// Number of Messages sent

	//	send_frame(0xFF, (i + 1), NULL, 0); /// Check Status After
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// SEND THREAD //////////////////////
////////////////////////////////////////////////////////////////////////////////
DWORD WINAPI ThreadFunc_snd(LPVOID lpParam)
{
	//int i;
	//i = 0;

	printf("SND thread\n");

	while (1) {
		////////////////////// EARLY THREAD TEST /////////////////
		//i++;
		//fprintf(stderr, "A %d\r", i);
		/////////////////////////////////////////////////////////
		if (flag_TxLoop) {
			testLoop();
			flag_TxLoop = FALSE;
		} else {
			//////////////////////////// AGAIN LOOP ////////////////
			if (flag_AgainLoop) {
				Sleep(10000);
				system("cls");
				printf("Loop Again\r\n");
				flag_TxLoop = TRUE;
			}
			////////////////////////////////////////////////////////
		}
		if (flag_Connect == FALSE) break;
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// RECEIVE THREAD ///////////////////
////////////////////////////////////////////////////////////////////////////////
DWORD WINAPI ThreadFunc_rcv(LPVOID lpParam)
{
	//int i;
	//i = 0;

	printf("RCV thread\n");

	while (1) {
		////////////////////// EARLY THREAD TEST /////////////////
		//i++;
		//fprintf(stderr, "B %d\r", i);
		/////////////////////////////////////////////////////////
		if (flag_Connect) {
			if (recieve_check()) flag_Connect = FALSE;
		}
		else {
			break;
		}
	}
	return 0;
}

void printHelp(void)
{
	printf("Press Key:\n");
	printf("d = Set Dummy Delay (ms)\n");
	printf("0 = Send Sync 0\n");
	printf("1 = Send Sync 1\n");
	printf("x = Configure Sync Params\n");
	printf("e = Get Group Report\n");
	printf("z = Get and Zero Errors and Max\n");
	printf("t = Tx Test Message 1\n");
	printf("s = Tx Test Message 2\n");
	printf("b = Induce Ethernet Error\n");
	printf("l = Loop Test (start/stop)\n");
	printf("i = Print Loop i step\n");
	printf("n = Print Rx Messages Count\n");
	printf("g = Toggle Print Format\n");
	printf("w = Run Loop + wait + run Loop again\n");
	printf("f = save log to file\n");
	printf("a = Get MAC Address\n");
	printf("y = Get Firmware Version\n");
	printf("r = -reserved-\n");
	printf("q = quit!\n");
	printf("Note: Use 'Ctrl-D' or 'Ctrl-C' to close connection\n");
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

/////////////////////////////////////////// SAVE TO FILE //////////////////////////////
void FileLogOpen()
{
	char		*fileName;
	errno_t		err_ret;

	log_file = NULL;

	fileName = "EtherCAN_log.txt";

	err_ret = fopen_s(&log_file, fileName, "w");

	if (err_ret != 0)
	{
		return;
	}
	printf(fileName);
	printf("\r\n");
	cuentaRx = 0;
	fputs("Begin\n", log_file);
	//fclose(log_file);

}
/////////////////////////////////////////// CLOSE FILE //////////////////////////////
void FileLogClose()
{
	char		texto[MAX_CHAR_GUP];

	if (log_file) {
		sprintf_s(texto, MAX_CHAR_GUP, "cuentaRx = %d ", cuentaRx);
		fputs(texto, log_file);
		//fputs("Done!", log_file);
		fclose(log_file);
		log_file = NULL;
	}
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// KEY-BOARD THREAD  ////////////////
////////////////////////////////////////////////////////////////////////////////
DWORD WINAPI ThreadFunc_key(LPVOID lpParam)
{
	//int i;
	//i = 0;

	int				in_c;
	uint8_t			data[8];
	uint8_t			bus_id;
	uint16_t		COB_ID;
	uint8_t			k;
	static uint8_t	test_diag=1;

	printf("Key thread\n");

	while (1) {
		////////////////////// EARLY THREAD TEST /////////////////
		//i++;
		//fprintf(stderr, "C %d\r", i);
		/////////////////////////////////////////////////////////
		int in_c = _getche();
		printf(" ");
		switch (in_c) {
		case 'd':
			if (dummyDelay) dummyDelay = 0; else dummyDelay = 1;
			printf("Dummy Delay %d\n", dummyDelay);
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
			data[0] = (1 << 1);				//(0x02);	 
			send_frame(10, 0, data, 1);
			data[0] = (1 << 1) | (1 << 5);	//0x03;
			send_frame(11, 0, data, 1);
			break;
		case 'e':
			printf("Get Error\n");
			flag_ShowErr = TRUE;
			send_frame(MSG_TYPE_GRUP, 0, data, 0); /// Get Group Report
			break;
		case 'z':
			printf("Get Error and reset Errors\n");
			flag_ShowErr = TRUE;
			data[0] = 0; /// DO NOT Zero Errors and Max
			data[0] = 1; /// Zero Errors and Max
			send_frame(MSG_TYPE_ERRS, 0, data, 1); /// Get Errors & Zero them
			break;
		case 't':
			uint8_t dataT[8] = { 7, 6, 5, 4, 3, 2, 1, 0 }; /// 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 is Ping, but here we add stuff so we know is ours!!
			bus_id = 5;
			COB_ID = 0x60A;
			send_frame(bus_id, COB_ID, dataT, 8);
			printf("Tx bus_id=%d COB-ID=0x%03X: data= ", bus_id, COB_ID);
			for (k = 0; k < 8; k++) printf("0x%02X ", dataT[k]);
			printf("\n");
			break;
		case 's':
			uint8_t dataS[8] = { 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 }; /// 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 is Ping, but here we add stuff so we know is ours!!													  
			bus_id = 1;
			COB_ID = 0x610;
			send_frame(bus_id, COB_ID, dataS, 8);
			printf("Tx bus_id=%d COB-ID=0x%03X: data= ", bus_id, COB_ID);
			for (k = 0; k < 8; k++) printf("0x%02X ", dataS[k]);
			printf("\n");
			break;
		case 'b':
			printf("Send 'Hola\r\n' to genereate RCV error!\n");
			send_ascii("Hola\r\n");
			break;
		case 'l':
			if (flag_TxLoop) flag_TxLoop = FALSE; else flag_TxLoop = TRUE;	/// toggle ///
			break;
		case '\004': // Ctrl-D
		case '\003': // Ctrl-C			
		case 'q':
			flag_Connect = FALSE;
			FileLogClose();
			break;
		case 'c':
			printf("loopCount=%d\n", loopCount);
			break;
		case 'i':
			flag_TxLoopi = TRUE;
			break;
		case 'n':
			printf("\r\nRx Count = %u\n", cuentaRx);
			cuentaRx = 0;
			break;
		case 'h':
			printHelp();
			break;
		case 'v':
			if (flag_Verbose) flag_Verbose = FALSE; else flag_Verbose = TRUE;	/// toggle ///
			printf("flag_Verbose= %d\n", flag_Verbose);
			break;
		case 'g':
			if (flag_Z) flag_Z = FALSE; else flag_Z = TRUE;	/// toggle ///
			printf("flag_Z= %d\n", flag_Z);
			break;
		case 'w':
			if (flag_AgainLoop) flag_AgainLoop = FALSE; else flag_AgainLoop = TRUE;	/// toggle ///
			if (flag_AgainLoop && (flag_TxLoop == FALSE)) flag_TxLoop = TRUE;
			printf("flag_AgainLoop= %d\n", flag_AgainLoop);
			break;
		case 'f':
			FileLogOpen();
			break;
		case 'y':
			flag_ShowErr = TRUE;
			send_frame(MSG_TYPE_FVER, 0, data, 0); /// Get Firmware Version
			break;
		case 'a':
			flag_ShowErr = TRUE;
			send_frame(MSG_TYPE_MACA, 0, data, 0); /// Get MAC Address
			break;
		case 'r':
			printf("\n");
			flag_ShowErr = TRUE;
			switch (test_diag) {
			case 1:		send_frame(MSG_TYPE_FVER, 0, data, 0); break;
			case 2:		send_frame(MSG_TYPE_MACA, 0, data, 0); break;
			case 3:		send_frame(MSG_TYPE_EBUF, 0, data, 0); break;
			case 4:		send_frame(MSG_TYPE_RXTX, 0, data, 0); break;
			case 5:		send_frame(MSG_TYPE_ETHC, 0, data, 0); break;

			case 6:		send_frame(MSG_TYPE_CRX0, 0, data, 0); break;
			case 7:		send_frame(MSG_TYPE_CRX1, 0, data, 0); break;
			case 8:		send_frame(MSG_TYPE_CRX2, 0, data, 0); break;
			case 9:		send_frame(MSG_TYPE_CRX3, 0, data, 0); break;
			case 10:	send_frame(MSG_TYPE_CRX4, 0, data, 0); break;
			case 11:	send_frame(MSG_TYPE_CRX5, 0, data, 0); break;

			
			case 12:	send_frame(MSG_TYPE_CTX0, 0, data, 0); break;
			case 13:	send_frame(MSG_TYPE_CTX1, 0, data, 0); break;
			case 14:	send_frame(MSG_TYPE_CTX2, 0, data, 0); break;
			case 15:	send_frame(MSG_TYPE_CTX3, 0, data, 0); break;
			case 16:	send_frame(MSG_TYPE_CTX4, 0, data, 0); break;
			case 17:	send_frame(MSG_TYPE_CTX5, 0, data, 0); break;

			case 18: send_frame(MSG_TYPE_ERRS, 0, data, 0); break;
			default: test_diag = 0;
				printf("Fin\n");
				break;
			}		
			test_diag++;
			break;
		default:
			printf("not valid\n");
		}

		if (flag_Connect == FALSE) break;
	}

	return 0;
}


//////////////////////////////////// HANDLE CONTROL-C //////////////////////
BOOL WINAPI HandlerRoutineX(_In_ DWORD dwCtrlType)
{
	fprintf(stderr, "HandlerRoutineX\r\n");

	if (dwCtrlType == CTRL_C_EVENT) {
		if (sock) closesocket(sock);
		WSACleanup();
		exit(0);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////// --------------------- MAIN ------------------- /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	int i;

	flag_Z			= FALSE;
	flag_AgainLoop	= FALSE;
	flag_ShowErr	= FALSE;

	//// set location of windows in screen ///
	/// intercept case WM_INITDIALOG mfc console application wopuld be a better place for this but ...
	HWND consoleWindow = GetConsoleWindow();
	SetWindowPos(consoleWindow, 0, 400, 300, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
	///////////////////////////////////////////

	SetConsoleCtrlHandler(HandlerRoutineX, TRUE);

	printHelp();

    if (argc < 3)
    {
        fprintf(stderr, "Usage: %s <ip address> <port>\n", argv[0]);
        return -1;
    }

	fprintf(stderr, "Connect to: <ip address> %s <port> %d\n", argv[1], atoi(argv[2]));
    
	sock = make_socket(argv[1], atoi(argv[2]));

    if (sock < 0)
    {
        return -1;
    }
	
	printf("Connected!!\r\n");

	flag_Connect = TRUE;

	///////////////////////// Threads //////////////////////////////////////////////
	DWORD   dwThreadIdArray[MAX_THREADS];
	HANDLE  hThreadArray[MAX_THREADS];

	for (int i = 0; i<MAX_THREADS; i++)
	{
		// Create the thread to begin execution on its own.

		switch (i) {
		case 0:
			hThreadArray[i] = CreateThread(NULL, 0, ThreadFunc_snd, NULL, 0, &dwThreadIdArray[i]);
			break;
		case 1:
			hThreadArray[i] = CreateThread(NULL, 0, ThreadFunc_rcv, NULL, 0, &dwThreadIdArray[i]);
			break;
		case 2:
			hThreadArray[i] = CreateThread(NULL, 0, ThreadFunc_key, NULL, 0, &dwThreadIdArray[i]);
			break;
		}

		// Check the return value for success.

		if (hThreadArray[i] == NULL)
		{
			MessageBox(NULL, L"Create Send-Thread failed", L"Error!", MB_ICONERROR);
			ExitProcess(3);
		}
	} // End of main thread creation loop.

	// Wait until all threads have terminated.

	WaitForMultipleObjects(MAX_THREADS, hThreadArray, TRUE, INFINITE);

	// Close all thread handles and free memory allocations.

	for (int i = 0; i<MAX_THREADS; i++)
	{
		CloseHandle(hThreadArray[i]);
	}

	closesocket(sock);
	WSACleanup();

	return 0;
}

