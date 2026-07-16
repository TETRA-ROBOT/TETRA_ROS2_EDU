#include "udp.h"

KanaviUDP::KanaviUDP(const std::string& localIP, const int& port, const std::string& multicastIP)
{
	// init Unicast
	if (-1 == init(localIP, port, multicastIP, true))
	{
		throw std::runtime_error("Failed to initialize UDP with multicast");
	}
}

KanaviUDP::KanaviUDP(const std::string& localIP, const int& port)
{
	// init Unicast
	if (-1 == init(localIP, port))
	{
		throw std::runtime_error("Failed to initialize UDP");
	}
}


KanaviUDP::~KanaviUDP()
{
}

int KanaviUDP::init(const std::string& ip, const int& port, std::string multicastIP, bool multiChecked)
{
	printf("\n");

	memset(mUdpBuf, 0, MAX_BUF_SIZE);

	mUdpSocket = socket(PF_INET, SOCK_DGRAM, 0);
	if (-1 == mUdpSocket)
	{
		perror("UDP Socket Failed");
		return -1;
	}

	memset(&mUdpAddr, 0, sizeof(sockaddr_in));
	if(!multiChecked) // unicast
	{
		printf("[UDP] Unicast Mode      : %s:%d\n", ip.c_str(), port);
		mUdpAddr.sin_family = AF_INET;
		mUdpAddr.sin_addr.s_addr = inet_addr(ip.c_str());
		mUdpAddr.sin_port = htons(port);
	}
	else		     // multicast
	{
		printf("[UDP] Local IP          : %s:%d\n", ip.c_str(), port);
		printf("[UDP] Multicast IP      : %s\n", multicastIP.c_str());
		mUdpAddr.sin_family = AF_INET;
		mUdpAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		mUdpAddr.sin_port = htons(port);
		mMultiAddr.imr_multiaddr.s_addr = inet_addr(multicastIP.c_str());
		mMultiAddr.imr_interface.s_addr = inet_addr(ip.c_str());
		setsockopt(mUdpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void*)&mMultiAddr, sizeof(mMultiAddr));
	}

	//time out
	struct timeval optVal = {1,500};
	int optlen = sizeof(optVal);
	setsockopt(mUdpSocket, SOL_SOCKET, SO_RCVTIMEO, &optVal, optlen);

	checkUdpBufSize();

	return 0;
}

void KanaviUDP::checkUdpBufSize()
{
	int sendSize;
	socklen_t optSize = sizeof(sendSize);
	getsockopt(mUdpSocket, SOL_SOCKET, SO_SNDBUF, &sendSize, &optSize);
	printf("[UDP] Send Buffer Size  : %d\n", sendSize);

	int recvSize;
	getsockopt(mUdpSocket, SOL_SOCKET, SO_RCVBUF, &recvSize, &optSize);
	printf("[UDP] Recv Buffer Size  : %d\n", recvSize);

	//reset udp buffer size
	int setSize = 0;

	if (sendSize < MAX_BUF_SIZE || recvSize < MAX_BUF_SIZE)
	{
		if(sendSize < recvSize)
		{
			setSize = recvSize;
		}
		else
		{
			setSize = sendSize;
		}

		if(sendSize < MAX_BUF_SIZE)
		{
			setsockopt(mUdpSocket, SOL_SOCKET, SO_SNDBUF, &setSize, sizeof(setSize));			
		}
		if(recvSize < MAX_BUF_SIZE)
		{
			setsockopt(mUdpSocket, SOL_SOCKET, SO_RCVBUF, &setSize, sizeof(setSize));
		}
	}
}

std::vector<uint8_t> KanaviUDP::GetData()
{
	memset(&mSenderAddr, 0, sizeof(struct sockaddr_in));
	socklen_t addrLen = sizeof(mSenderAddr);

	std::vector<uint8_t> output;

	int size = recvfrom(mUdpSocket, mUdpBuf, MAX_BUF_SIZE, 0, (struct sockaddr*)&mSenderAddr, &addrLen);

	char ipStr[INET_ADDRSTRLEN];
    // IP 주소 변환 (network byte order → string)
    inet_ntop(AF_INET, &(mSenderAddr.sin_addr), ipStr, sizeof(ipStr));

	if(size > 0)
	{
		output.resize(size);
		std::copy(mUdpBuf, mUdpBuf + size, output.begin());
	}

	return output;
}

void KanaviUDP::SendData(std::vector<uint8_t> data)
{
	printf("NOT ACTIVATED...\n");
}

int KanaviUDP::Connect()
{
	return bind(mUdpSocket, (struct sockaddr*)&mUdpAddr, sizeof(mUdpAddr));
}

int KanaviUDP::Disconnect()
{
	return close(mUdpSocket);
}