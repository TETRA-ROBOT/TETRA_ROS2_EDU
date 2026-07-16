#include "ros2/kanavi_node.h"
using namespace std::chrono_literals;  // "10ms"와 같은 단위 사용을 위해 필요

KanaviNode::KanaviNode(const std::string &node, int &argc, char **argv) : rclcpp::Node(node)
{
	mbCheckedMulticast = false;

	// parse PARAMETERS
	mArgv = std::make_unique<ArgvParser>(argc, argv);

	// get PARAMETERS
	ArgvContainer argvs = mArgv->GetParameters();

	// set PARAMETERS
	mLocalIP = argvs.localIP;
	mPort = argvs.port;
	
	mTopicName = argvs.topicName;
	mFixedName = argvs.fixedName;
	mbCheckedMulticast = argvs.checkedMulticast;
	mbCheckedDebug = argvs.checkedDebug || argvs.checkedTimestamp;
	mbCheckedTimestamp = argvs.checkedTimestamp;

	if(mbCheckedMulticast)
	{
		mMulticastIP = argvs.multicastIP;
		// init UDP network
		mPtrUDP = std::make_unique<KanaviUDP>(mLocalIP, mPort, mMulticastIP);
	}
	else
	{
		// init UDP network
		mPtrUDP = std::make_unique<KanaviUDP>(mLocalIP, mPort);
	}

	if(-1 == mPtrUDP->Connect())
	{
		return;
	}

	setLogParameters();

	// check model using node name
	int model = -1;
	if (!strcmp("r270", node.c_str()))
	{
		model = kanavi::common::protocol::eModel::R270;
	}
	else if (!strcmp("r4", node.c_str()))
	{
		model = kanavi::common::protocol::eModel::R4;
	}
	else if (!strcmp("r2", node.c_str()))
	{
		model = kanavi::common::protocol::eModel::R2;
	}
	mRotateAngle = kanavi::common::specification::BASE_ZERO_ANGLE;

	if (model < 0)
	{
		return;
	}

	CalculateAngular(model);

	// init LiDAR Processor 
	mPtrProcess = std::make_unique<KanaviLidar>(model);

	mPtrPointCloud.reset(new PointCloudT);

	// init
	auto qosProfile = rclcpp::QoS(rclcpp::KeepLast(10));
	mPtrPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName, qosProfile);

	// active UDP RECV using timer
	mPtrTimer = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&KanaviNode::receiveData, this));
}

KanaviNode::~KanaviNode()
{
	if (mPtrUDP) 
	{ 
		mPtrUDP->Disconnect();
	}
	if (mPtrTimer) 
	{
		mPtrTimer->cancel();
	}
}

void KanaviNode::receiveData()
{
	// recv data using udp
	std::vector<uint8_t> recvBuf = mPtrUDP->GetData();
	
	if(!recvBuf.empty())
	{
		mPtrProcess->Process(recvBuf);
	}

	if(mPtrProcess->IsProcessEnd())
	{
		mPtrProcess->InitProcessEnd();

		length2PointCloud(mPtrProcess->GetDatagram());

		rotateAxisZ(mPtrPointCloud, mRotateAngle);

		if(mbCheckedDebug)
		{
			if(mbCheckedTimestamp)
			{
				time_t rawtime = time(nullptr); 
				struct tm* timeinfo = localtime(&rawtime);
				printf("[%02d:%02d:%02d] -- KANAVI PROCESS --\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
			}
			else
			{
				printf("----- KANAVI PROCESS -----\n");
			}
			
			for (size_t i = 0; i < mPtrProcess->GetDatagram().GetChannelCount(); ++i) 
			{
				if (mPtrProcess->GetDatagram().GetActiveChannels()[i]) 
				{
					printf("[NODE] PUBLISHING : %zuCH\n", i+1);
				}
			}
		}
		mPtrProcess->GetDatagram().GetActiveChannels().assign(mPtrProcess->GetDatagram().GetChannelCount(), false);  

		publishPointCloud(mPtrPointCloud);

		mPtrPointCloud->clear();
	}

	recvBuf.clear();
}

void KanaviNode::endProcess()
{
	// Clean up resources safely
	this->mPtrTimer->cancel();
	// Signal to stop spinning
	rclcpp::shutdown();
}

/**
 * @brief output Log for LiDAR & ROS Node Information
 * 
 */
void KanaviNode::setLogParameters()
{
	printf("--------------------------------------\n");
	printf("           KANAVI ROS2\n");
	printf("--------------------------------------\n");
	printf("Local IP         : %s\n", mLocalIP.c_str());
	printf("Port Number      : %d\n", mPort);
	if (mbCheckedMulticast)
	{
		printf("Multicast IP     : %s\n", mMulticastIP.c_str());
	}
	printf("Fixed Frame Name : %s\n", mFixedName.c_str());
	printf("Topic Name       : %s\n", mTopicName.c_str());
	printf("--------------------------------------\n");
}

void KanaviNode::CalculateAngular(int model)
{
	switch (model)
	{
	case kanavi::common::protocol::eModel::R2:
		for (int i = 0; i < kanavi::common::specification::R2::VERTICAL_CHANNEL; i++)
		{
			mVecVerticalSin.push_back(sin(DEG2RAD(kanavi::common::specification::R2::VERTICAL_RESOLUTION * i)));
			mVecVerticalCos.push_back(cos(DEG2RAD(kanavi::common::specification::R2::VERTICAL_RESOLUTION * i)));
		}
		for (int i = 0; i < kanavi::common::specification::R2::HORIZONTAL_DATA_CNT; i++)
		{
			mVecHorizontalSin.push_back(sin(DEG2RAD(kanavi::common::specification::R2::HORIZONTAL_RESOLUTION * i)));
			mVecHorizontalCos.push_back(cos(DEG2RAD(kanavi::common::specification::R2::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	case kanavi::common::protocol::eModel::R4:
		for (int i = 0; i < kanavi::common::specification::R4::VERTICAL_CHANNEL; i++)
		{
			mVecVerticalSin.push_back(sin(DEG2RAD(kanavi::common::specification::R4::VERTICAL_RESOLUTION * i)));
			mVecVerticalCos.push_back(cos(DEG2RAD(kanavi::common::specification::R4::VERTICAL_RESOLUTION * i)));
		}

		for (int i = 0; i < kanavi::common::specification::R4::HORIZONTAL_DATA_CNT; i++)
		{
			mVecHorizontalSin.push_back(sin(DEG2RAD(kanavi::common::specification::R4::HORIZONTAL_RESOLUTION * i)));
			mVecHorizontalCos.push_back(cos(DEG2RAD(kanavi::common::specification::R4::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	case kanavi::common::protocol::eModel::R270:
		for (int i = 0; i < kanavi::common::specification::R270::HORIZONTAL_DATA_CNT; i++)
		{
			mVecHorizontalSin.push_back(sin(DEG2RAD(kanavi::common::specification::R270::HORIZONTAL_RESOLUTION * i)));
			mVecHorizontalCos.push_back(cos(DEG2RAD(kanavi::common::specification::R270::HORIZONTAL_RESOLUTION * i)));
		}
		break;
	}
}

void KanaviNode::length2PointCloud(KanaviDatagram datagram)
{
	// generate Point Cloud
	generatePointCloud(datagram, *mPtrPointCloud);
}

void KanaviNode::generatePointCloud(const KanaviDatagram& datagram, PointCloudT& cloud)
{
	switch (datagram.GetModel())
	{
	case kanavi::common::protocol::eModel::R2:
		for (int ch = 0; ch < kanavi::common::specification::R2::VERTICAL_CHANNEL; ch++)
		{
			for (int index = 0; index < kanavi::common::specification::R2::HORIZONTAL_DATA_CNT; index++)
			{
				cloud.push_back(length2Point(datagram.GetLengthBuffer()[ch][index], mVecVerticalSin[ch], mVecVerticalCos[ch], mVecHorizontalSin[index], mVecHorizontalCos[index]));
			}
		}
		break;
	case kanavi::common::protocol::eModel::R4:
		for (int ch = 0; ch < kanavi::common::specification::R4::VERTICAL_CHANNEL; ch++)
		{
			for (int index = 0; index < kanavi::common::specification::R4::HORIZONTAL_DATA_CNT; index++)
			{
				cloud.push_back(length2Point(datagram.GetLengthBuffer()[ch][index], mVecVerticalSin[ch], mVecVerticalCos[ch], mVecHorizontalSin[index], mVecHorizontalCos[index]));
			}
		}
		break;
	case kanavi::common::protocol::eModel::R270:
		for (int index = 0; index < kanavi::common::specification::R270::HORIZONTAL_DATA_CNT; index++)
		{
			cloud.push_back(length2Point(datagram.GetLengthBuffer()[0][index], 0, 1, mVecHorizontalSin[index], mVecHorizontalCos[index]));
		}
		break;
	default:
		assert(false && "Unknown model in generatePointCloud");
		break;
	}
}

PointT KanaviNode::length2Point(float len, float vSin, float vCos, float hSin, float hCos)
{
	pcl::PointXYZRGB point;

	point.x = len * vCos * hCos;
	point.y = len * vCos * hSin;
	point.z = len * vSin;

	float r, g, b;
	hsv2Rgb(&r, &g, &b, len * 20.0f, 1.0f, 1.0f); // convert hsv to rgb
	point.r = r * 255.f;
	point.g = g * 255.f;
	point.b = b * 255.f;

	return point;
}

void KanaviNode::hsv2Rgb(float* fR, float* fG, float* fB, float fH, float fS, float fV)
{
	float fC = fV * fS;
	float fHPrime = fmod(fH / 60.0f, 6.0f);
	float fX = fC * (1 - fabs(fmod(fHPrime, 2.0f) - 1.0f));

	if (0 <= fHPrime && fHPrime < 1)
	{
		*fR = fC;
		*fG = fX;
		*fB = 0.0f;
	}

	else if (0 <= fHPrime && fHPrime < 2)
	{
		*fR = fX;
		*fG = fC;
		*fB = 0.0f;
	}
	else if (0 <= fHPrime && fHPrime < 3)
	{
		*fR = 0.0f;
		*fG = fC;
		*fB = fX;
	}
	else if (0 <= fHPrime && fHPrime < 4)
	{
		*fR = 0.0f;
		*fG = fX;
		*fB = fC;
	}
	else if (0 <= fHPrime && fHPrime < 5)
	{
		*fR = fX;
		*fG = 0.0f;
		*fB = fC;
	}
	else if (0 <= fHPrime && fHPrime < 6)
	{
		*fR = fC;
		*fG = 0.0f;
		*fB = fX;
	}
	else
	{
		*fR = 0.0f;
		*fG = 0.0f;
		*fB = 0.0f;
	}
}

void KanaviNode::rotateAxisZ(PointCloudT::Ptr cloud, float angle)
{
	float rad = pcl::deg2rad(angle);
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	matrix(0, 0) = cos(rad);
	matrix(0, 1) = -sin(rad);
	matrix(1, 0) = sin(rad);
	matrix(1, 1) = cos(rad);

	pcl::transformPointCloud(*cloud, *cloud, matrix);
}

void KanaviNode::publishPointCloud(PointCloudT::Ptr cloud)
{
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);

	msg.header.set__frame_id(mFixedName);	// rviz의 fixed frame을 따라가야함
	msg.header.set__stamp(this->get_clock()->now());

	mPtrPublisher->publish(msg);
}