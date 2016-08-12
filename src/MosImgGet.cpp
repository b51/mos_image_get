#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <mos_image_get/MosImgGet.h>

#define INST_CAPT_IMAGE 0x01
#define INST_SEND_IMAGE 0x02
#define MAX_WIRELESS_RECEIVE 4096
#define INST_IDENTITY 0xff
#define GET_RAW_IMAGE_PORT 50002

using namespace std;

bool tcp_client::CaptImgReqSend()
{
    int i;
    unsigned char pcSendBuffer[sizeof(JPG_SEND_PACKET)];
    unsigned int checksumCal = 0;
    infoSended = (JPG_SEND_PACKET *)pcSendBuffer;

    memset(&server_addr,0,sizeof(server_addr));
    memset(pcSendBuffer,0,sizeof(pcSendBuffer));

    infoSended->identity = INST_IDENTITY;
    memcpy(infoSended->version, "MOS", 3);
    infoSended->length = sizeof(JPG_SEND_PACKET);
    infoSended->inst_info = INST_CAPT_IMAGE;

    for(i=0; i<sizeof(JPG_SEND_PACKET)-sizeof(unsigned int); i++)
    {
        checksumCal = checksumCal + pcSendBuffer[i];
    }
    infoSended->checksum = checksumCal;

    if( send( socket_fd,pcSendBuffer,sizeof(pcSendBuffer),0 ) < 0 )
    {
        printf("send message error\n");
        exit(0);
    }
    return true;
}

bool tcp_client::RobImgGet()
{
    int i;
    unsigned int j;
    unsigned int lenImgDataLeft;
    unsigned int lenReceived = 0; 
    unsigned int iHeadDataRecv;
    unsigned int iImgDataRecv;
    unsigned char info[sizeof(JPG_RECV_PACKET)];
    unsigned char jpgPackRecv[sizeof(JpgBuffer)];

    jpgBuffer = (JpgBuffer *)jpgPackRecv;
    numReceived = 0;

    iHeadDataRecv = recv(socket_fd,info,sizeof(JPG_RECV_PACKET),0); 

    if (iHeadDataRecv <= 0)
    {
        printf("receive image error\n");
        close(socket_fd);
        socket_fd = -1;
        exit(0);
    }

    for(i=0;i<iHeadDataRecv;i++)
    {
    	if(PacketReconstruct(info[i]))
    	{
    		if(PacketCheckSum())
    		{
                lenImgDataLeft = sizeof(jpgPackRecv);
                do
                {
                    iImgDataRecv = recv(socket_fd,jpgPackRecv+lenReceived,lenImgDataLeft,0);
                    lenReceived += iImgDataRecv;
                    lenImgDataLeft = lenImgDataLeft - iImgDataRecv;
                }while(lenImgDataLeft > 0);
                printf("lenReceived = %d\n", lenReceived);
    		}
    	}
    }	
    if (jpgBuffer->updated)
    {
        return true;
    }
    else
        return false;
}

bool tcp_client::PacketCheckSum(void)
{
	int i;
	unsigned int checksumCal=0;
	JPG_RECV_PACKET *buffer;
	buffer=(JPG_RECV_PACKET *)bufRecvToProc;
	for(i=0;i<(sizeof(JPG_RECV_PACKET)-sizeof(unsigned int));i++)
	{
		checksumCal=checksumCal+bufRecvToProc[i];	
	}
	if(buffer->checksum==checksumCal)	
	{
		return true;
	}

	return true;
}

bool tcp_client::PacketReconstruct(unsigned char beReceived)
{
	if(numReceived>=4)
	{
		JPG_RECV_PACKET *info;
		info=(JPG_RECV_PACKET *)bufRecv;
		if(numReceived==(unsigned int)info->length-1)
		{
			//reconstruct final
			bufRecv[numReceived++]=beReceived;
			memcpy(bufRecvToProc,bufRecv,numReceived);
			memset(bufRecv,0,sizeof(bufRecv));
			numReceived=0;
			return true;
		}else if(numReceived<(unsigned int)info->length-1)
		{
			//reconstruct write
			bufRecv[numReceived++]=beReceived;
		}
        else
        {
			bufRecv[numReceived++]=beReceived;
        }
	}else if(numReceived>=3)
	{
		if(beReceived=='S')
		{
			bufRecv[numReceived++]=beReceived;
		}else
		{
			numReceived=0;
			return false;
		}
	}else if(numReceived>=2)
	{
		if(beReceived=='O')
		{
			bufRecv[numReceived++]=beReceived;
		}else
		{
			numReceived=0;
			return false;
		}
	}else if(numReceived>=1)
	{
		if(beReceived=='M')
		{
			bufRecv[numReceived++]=beReceived;
		}else
		{
			numReceived=0;
			return false;
		}
	}else
	{
		if(beReceived==0xff)
		{
			bufRecv[numReceived++]=beReceived;
		}else
		{
			numReceived=0;
			return false;
		}
	}
	
	return false;
}
 
void tcp_client::tcp_client_connect(char* server_ip)
{
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(GET_RAW_IMAGE_PORT);

    if( (socket_fd = socket(AF_INET,SOCK_STREAM,0)) < 0 )
    {
        printf("create socket error: %s(errno:%d)\n)",strerror(errno),errno);
        exit(0);
    }

    if( inet_pton(AF_INET,server_ip,&server_addr.sin_addr) <=0 )
    {
        printf("inet_pton error for %s\n",server_ip);
        exit(0);
    }

    if( connect(socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr))<0)
    {
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0);
    }
    
}

void tcp_client::tcp_client_disconnect()
{
    close(socket_fd);
}

//tcp_client::get_raw_image(char *pbuf)
//{
//    tcp_client(MOS_IP, GET_RAW_IMAGE_PORT);
//    memcpy(pbuf,rawImageBuf,RAW_IMAGE_SIZE)
//}

//int main(int argc,char* argv[])
//{
//    tcp_client tc(argv[1],argv[2]);
//    return 0;
//}

