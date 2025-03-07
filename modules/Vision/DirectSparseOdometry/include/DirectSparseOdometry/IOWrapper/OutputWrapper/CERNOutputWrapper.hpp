/* @Author: Maximilian Enthoven
 * @Copyright CERN 2018
 */


#pragma once

#include "DirectSparseOdometry/util/MinimalImage.hpp"
#include "DirectSparseOdometry/IOWrapper/Output3DWrapper.hpp"

#include "DirectSparseOdometry/FullSystem/HessianBlocks.hpp"
#include "DirectSparseOdometry/util/FrameShell.hpp"

#include <fstream>
#include <iomanip>
#include <memory>

#include <sophus/se3.hpp>


namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

class CERNOutputWrapper : public Output3DWrapper
{
public:
	
        CERNOutputWrapper(bool live, std::shared_ptr<std::vector<long>> timestamps);
	
	void getDepth();

        virtual ~CERNOutputWrapper();
        virtual void publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity);

        virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib);

        
        
        virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib);

        virtual void pushLiveFrame(FrameHessian* image);

        virtual void pushDepthImage(MinimalImageB3* image);
        virtual bool needPushDepthImage();

        virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF );
        
	
private:
  
	int _pccounter;
	std::ofstream _savepts;
	bool _live;
	float _x,_y,_z;
	long long _timestamp;

	std::shared_ptr<std::vector<long>> _timestampVector;
	
	std::string long2string(long num)
	{
		std::stringstream ss;
		ss << num;
		return ss.str();
	}
	
};

}

}
