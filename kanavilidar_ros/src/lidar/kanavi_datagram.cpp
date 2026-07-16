#include "kanavi_datagram.h"
#include <cassert>

KanaviDatagram::KanaviDatagram(int model){
	mModel = model;
	switch(mModel)
	{
	case kanavi::common::protocol::eModel::R2:
		mVerticalFOV = kanavi::common::specification::R2::VERTICAL_FOV;
		mVerticalResolution = kanavi::common::specification::R2::VERTICAL_RESOLUTION;
		mHorizontalFOV = kanavi::common::specification::R2::HORIZONTAL_FOV;
		mHorizontalResolution = kanavi::common::specification::R2::HORIZONTAL_RESOLUTION;
		mChannelPointsSize = kanavi::common::specification::R2::HORIZONTAL_DATA_CNT;
		mChannelCount = kanavi::common::specification::R2::VERTICAL_CHANNEL;
		mVecRawBuf.resize(kanavi::common::specification::R2::VERTICAL_CHANNEL);
		mVecLenBuf.resize(kanavi::common::specification::R2::VERTICAL_CHANNEL);
		for (auto& buf : mVecLenBuf) {
			buf.resize(kanavi::common::specification::R2::HORIZONTAL_DATA_CNT, 0.0f);
		}
		break;
	case kanavi::common::protocol::eModel::R4:
		mVerticalFOV = kanavi::common::specification::R4::VERTICAL_FOV;
		mVerticalResolution = kanavi::common::specification::R4::VERTICAL_RESOLUTION;
		mHorizontalFOV = kanavi::common::specification::R4::HORIZONTAL_FOV;
		mHorizontalResolution = kanavi::common::specification::R4::HORIZONTAL_RESOLUTION;
		mChannelPointsSize = kanavi::common::specification::R4::HORIZONTAL_DATA_CNT;
		mChannelCount = kanavi::common::specification::R4::VERTICAL_CHANNEL;
		mVecRawBuf.resize(kanavi::common::specification::R4::VERTICAL_CHANNEL);
		mVecLenBuf.resize(kanavi::common::specification::R4::VERTICAL_CHANNEL);
		for (auto& buf : mVecLenBuf) {
			buf.resize(kanavi::common::specification::R4::HORIZONTAL_DATA_CNT, 0.0f);
		}
		break;
	case kanavi::common::protocol::eModel::R270:
		mVerticalFOV = kanavi::common::specification::R270::VERTICAL_FOV;
		mVerticalResolution = kanavi::common::specification::R270::VERTICAL_RESOLUTION;
		mHorizontalFOV = kanavi::common::specification::R270::HORIZONTAL_FOV;
		mHorizontalResolution = kanavi::common::specification::R270::HORIZONTAL_RESOLUTION;
		mChannelPointsSize = kanavi::common::specification::R270::HORIZONTAL_DATA_CNT;
		mChannelCount = kanavi::common::specification::R270::VERTICAL_CHANNEL;
		mVecRawBuf.resize(kanavi::common::specification::R270::VERTICAL_CHANNEL);
		mVecLenBuf.resize(kanavi::common::specification::R270::VERTICAL_CHANNEL);
		for (auto& buf : mVecLenBuf) {
			buf.resize(kanavi::common::specification::R270::HORIZONTAL_DATA_CNT, 0.0f);
		}
		break;
	default:
		assert(false && "unknown model");
		break;
	}

	mVecChannelActive.assign(mChannelCount, false);  
}

KanaviDatagram::~KanaviDatagram()
{
}

int KanaviDatagram::GetModel() const 
{ 
    return mModel; 
}

double KanaviDatagram::GetVerticalFov() const 
{ 
    return mVerticalFOV; 
}


double KanaviDatagram::GetVerticalResolution() const 
{ 
    return mVerticalResolution; 
}
    
double KanaviDatagram::GetHorizontalFov() const 
{ 
    return mHorizontalFOV; 
}

double KanaviDatagram::GetHorizontalResolution() const 
{ 
    return mHorizontalResolution; 
}


bool KanaviDatagram::IsCheckedEnd() const 
{ 
    return mbCheckEnd; 
}

void KanaviDatagram::SetCheckedEnd(bool checked) 
{ 
    mbCheckEnd = checked; 
}

const std::string& KanaviDatagram::GetLidarIP() const 
{ 
    return mLidarIP; 
}

void KanaviDatagram::SetLidarIP(const std::string& ip) 
{ 
    mLidarIP = ip; 
}
const std::vector<std::vector<uint8_t>>& KanaviDatagram::GetRawBuffer() const 
{ 
    return mVecRawBuf; 
}

std::vector<std::vector<uint8_t>>& KanaviDatagram::GetRawBuffer() 
{ 
    return mVecRawBuf; 
}
    
const std::vector<std::vector<float>>& KanaviDatagram::GetLengthBuffer() const 
{ 
    return mVecLenBuf; 
}

std::vector<std::vector<float>>& KanaviDatagram::GetLengthBuffer() 
{ 
    return mVecLenBuf; 
}

size_t KanaviDatagram::GetChannelPointsSize() const 
{ 
    return mChannelPointsSize; 
}

uint8_t KanaviDatagram::GetChannelCount() const 
{ 
    return mChannelCount; 
}

const std::vector<bool>& KanaviDatagram::GetActiveChannels() const 
{ 
    return mVecChannelActive; 
}

std::vector<bool>& KanaviDatagram::GetActiveChannels() 
{ 
    return mVecChannelActive; 
}