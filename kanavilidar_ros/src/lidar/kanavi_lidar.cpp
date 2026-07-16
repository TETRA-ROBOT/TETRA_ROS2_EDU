#include "kanavi_lidar.h"

/**
 * @brief Construct a new kanavi lidar::kanavi lidar object
 *
 * @param model LiDAR Model ref include/common.h
 */
KanaviLidar::KanaviLidar(int model)
{
	mPtrdatagram = std::make_unique<KanaviDatagram>(model);
	mCheckedModel = -1;
	mCheckedChannel = 0;
	mTotalSize = 0;

	mbCheckedLidarInputed = false;
	mbCheckOnGoing = false;
	mbCheckParseEnd = false;
}

/**
 * @brief Destroy the kanavi lidar::kanavi lidar object
 *
 */
KanaviLidar::~KanaviLidar()
{
}

/**
 * @brief check LiDAR Sensor
 *
 * @param data
 * @return int
 */
int KanaviLidar::classification(const std::vector<uint8_t>& data)
{
	if ((data[kanavi::common::protocol::position::HEADER] & 0xFF) == kanavi::common::protocol::HEADER) // industrial header detected
	{
		uint8_t productline = data[kanavi::common::protocol::position::PRODUCTLINE];
		switch (productline)
		{
		case kanavi::common::protocol::eModel::R2:
            printf("[LIDAR] Model      : R2\n");
			return kanavi::common::protocol::eModel::R2;
		case kanavi::common::protocol::eModel::R4:
            printf("[LIDAR] Model      : R4\n");
			return kanavi::common::protocol::eModel::R4;
		case kanavi::common::protocol::eModel::R270:
            printf("[LIDAR] Model      : R270\n");
			return kanavi::common::protocol::eModel::R270;
		default:
			return -1;
		}
	}

	return -1;  // Invalid header - classification failed
}

int KanaviLidar::Process(const std::vector<uint8_t>& data)
{
	uint8_t mode = data[static_cast<int>(kanavi::common::protocol::position::eCommand::MODE)];
	uint8_t ch = data[static_cast<int>(kanavi::common::protocol::position::eCommand::PARAMETER)];

	// header value Check in data
	if ((data[kanavi::common::protocol::position::HEADER] & 0xFF) != kanavi::common::protocol::HEADER) 
	{
		if(mbCheckOnGoing)
		{
			r270(data, mPtrdatagram.get());
			return 0;
		}
		else
		{
			printf("[LiDAR] Invalid header\n");
			return -1;
		}
	}

	// check Model
	if (!mbCheckedLidarInputed)
	{
		mCheckedModel = classification(data);
		if (mCheckedModel == -1)
		{
			perror("LiDAR Classification Error!");
			return -1;
		}

		// check Model, one more time
		if (mCheckedModel != mPtrdatagram->GetModel())
		{
			perror("LiDAR Model not Matched");
			return -1;
		}
		
        printf("[LIDAR] Publishing : Started\n\n");

		mbCheckedLidarInputed = true;
		mCheckedChannel =  static_cast<int>(ch & 0x0F);
	}
	
	// Process data based on mode
	if (mode == kanavi::common::protocol::command::eMode::DISTANCE_DATA) 
	{
		try 
		{
			// Process the data directly
			switch (mCheckedModel) 
			{
				case kanavi::common::protocol::eModel::R2:
					r2(data, mPtrdatagram.get());
					break;
				case kanavi::common::protocol::eModel::R4:
					r4(data, mPtrdatagram.get());
					break;
				case kanavi::common::protocol::eModel::R270:
					r270(data, mPtrdatagram.get());
					break;
				default:
					printf("[LiDAR] Unknown model\n");
					return -1;
			}
			
		} 
		catch (const std::exception& e) 
		{
			printf("[LiDAR] Error in process: %s\n", e.what());
			return -1;
		}
	}
	return 0;
}

void KanaviLidar::parse(const std::vector<uint8_t> &data)
{
	if (data.size() < kanavi::common::protocol::position::RAWDATA_START)
	{
		printf("[LiDAR] Invalid data size\n");
		return;
	}

	switch (mPtrdatagram->GetModel())
	{
	case kanavi::common::protocol::R2:
		r2(data, mPtrdatagram.get());
		break;
	case kanavi::common::protocol::R4:
		r4(data, mPtrdatagram.get());
		break;
	case kanavi::common::protocol::R270:
		r270(data, mPtrdatagram.get());
		break;
	default:
		printf("[LiDAR] Unknown model\n");
		return;
	}
}

void KanaviLidar::r2(const std::vector<uint8_t>& input, KanaviDatagram* output)
{
	uint8_t mode = input[static_cast<int>(kanavi::common::protocol::position::eCommand::MODE)];
	uint8_t ch = input[static_cast<int>(kanavi::common::protocol::position::eCommand::PARAMETER)];
	int channel = static_cast<int>(ch & 0x0F);

	if (mode == kanavi::common::protocol::command::eMode::DISTANCE_DATA)
	{
		parseLength(input, output, channel); // convert byte to length
	}
}

void KanaviLidar::r4(const std::vector<uint8_t>& input, KanaviDatagram* output)
{
	try 
	{
		uint8_t ch = input[static_cast<int>(kanavi::common::protocol::position::eCommand::PARAMETER)];
		size_t channel = static_cast<size_t>(ch & 0x0F);

		// Validate channel number
		if (channel >= kanavi::common::specification::R4::VERTICAL_CHANNEL) 
		{
			return;
		}

		// Ensure len_buf is properly initialized
		if (output->GetLengthBuffer().empty() || output->GetLengthBuffer().size() <= channel)
		{
			printf("[LiDAR] len_buf not properly initialized for channel %zu\n", channel);
			return;
		}

		// process the current channel data
		parseLength(input, output, static_cast<int>(channel));
	} 
	catch (const std::exception& e) 
	{
		printf("[LiDAR] Error in r4: %s\n", e.what());
	}
}

void KanaviLidar::r270(const std::vector<uint8_t>& input, KanaviDatagram* output)
{
	// 첫 번째 패킷인 경우 (채널값 + 데이터)
	if (mVecTempBuf.empty()) // == !checked_onGoing
	{
		mbCheckOnGoing = true;
		
		// 첫 번째 패킷 데이터 저장
		mVecTempBuf = input;
	}
	// 두 번째 패킷인 경우 (나머지 데이터)
	else
	{
		// 두 번째 패킷은 헤더 체크를 하지 않음
		// 두 패킷의 데이터를 합쳐서 처리
		std::vector<uint8_t> combinedData;
		combinedData.insert(combinedData.end(), mVecTempBuf.begin(), mVecTempBuf.end());
		combinedData.insert(combinedData.end(), input.begin(), input.end());
		
		// 합쳐진 데이터 처리
		parseLength(combinedData, output, 0);
		
		// 다음 프레임을 위해 초기화
		mVecTempBuf.clear();
		mbCheckOnGoing = false;
	}
}

void KanaviLidar::parseLength(const std::vector<uint8_t>& input, KanaviDatagram* output, int ch)
{
	const size_t start = kanavi::common::protocol::position::RAWDATA_START;
	const size_t mExpectedPoints = output->GetChannelPointsSize();
	const size_t expectedBytes = mExpectedPoints * 2;

	// 입력 데이터 크기 검증
	if (start + expectedBytes > input.size()) 
	{
		printf("[LiDAR] Invalid data size for model %d: expected %zu bytes, got %zu\n", 
			mCheckedModel, expectedBytes, input.size() - start);
		return;
	}

	std::vector<float> convert;
	convert.reserve(mExpectedPoints);

	for (size_t i = start; i < start + expectedBytes; i += 2) 
	{
		int up = static_cast<int>(input[i]);
		int low = static_cast<int>(input[i + 1]);
		float len = static_cast<float>(up) + static_cast<float>(low) / 100.0f;
		convert.push_back(len);
	}

	if (static_cast<size_t>(ch) < output->GetLengthBuffer().size()) 
	{
		output->GetLengthBuffer()[ch] = std::move(convert);
		output->GetActiveChannels()[ch] = true;
	
		if(ch == mCheckedChannel)
        {
            mbCheckParseEnd = true;
        }
	}
}

bool KanaviLidar::IsProcessEnd() const
{
	return mbCheckParseEnd;
}

void KanaviLidar::InitProcessEnd()
{
	mbCheckParseEnd = false;
}

KanaviDatagram KanaviLidar::GetDatagram()
{
	return *mPtrdatagram;
}