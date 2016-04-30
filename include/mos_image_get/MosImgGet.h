#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

struct JpgBuffer
{
    char buf[640*480*2];
    int size;
    int updated;
};

struct JPG_SEND_PACKET
{
    unsigned char identity;
    char version[3];
    unsigned char length;
    unsigned char inst_info;
    unsigned int checksum;
};

struct JPG_RECV_PACKET
{
    unsigned char identity;
    char version[3];
    unsigned char length;
    unsigned char inst_info;
    unsigned int checksum;
};

class tcp_client
{
    private:
        int socket_fd;
        int numReceived;
        JPG_SEND_PACKET *infoSended;
        char bufRecv[sizeof(JPG_RECV_PACKET)];
        char bufRecvToProc[sizeof(JPG_RECV_PACKET)];
        struct sockaddr_in server_addr;
        bool PacketReconstruct(unsigned char beReceived);
        bool PacketCheckSum();
    public:
        JpgBuffer *jpgBuffer;
        bool CaptImgReqSend();
        bool RobImgGet();
        void tcp_client_connect(char* server_ip);
        void tcp_client_disconnect();
};
