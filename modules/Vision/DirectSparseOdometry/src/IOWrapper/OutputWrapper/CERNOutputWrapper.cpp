/* @Author: Maximilian Enthoven
 * @Copyright CERN 2017
 */


#include "DirectSparseOdometry/IOWrapper/OutputWrapper/CERNOutputWrapper.hpp"


namespace dso
{
	namespace IOWrap
	{
  
		std::ofstream trajectory;
		
		CERNOutputWrapper::CERNOutputWrapper(bool live, std::shared_ptr<std::vector<long>> timestamps)
		{
			printf("CREATED CERN OUTPUT WRAPPER\n");
			this-> _timestampVector = timestamps;
			this->_live = live;
			this->_pccounter = 0;
			
			bool sysRet = system("rm trunk/xyz.txt");

		}

		CERNOutputWrapper::~CERNOutputWrapper()
		{
			trajectory.close();
			printf("DESTROYED CERN OUTPUT WRAPPER\n");
		}

		void CERNOutputWrapper::publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity)
		{
		}

		
		// only executed everytime new KF is pushed
		void CERNOutputWrapper::publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) 
		{
			std::cout << "CERN: NUMBER OF KF: " << frames.size() << std::endl;   
			for(FrameHessian* f : frames)
			{  
			
				int id = f->frameID;
				float fx = HCalib->fxl();
				float fy = HCalib->fyl();
				float cx = HCalib->cxl();
				float cy = HCalib->cyl();
				float fxi = 1/fx;
				float fyi = 1/fy;
				float cxi = -cx / fx;
				float cyi = -cy / fy;
				
				const Eigen::Matrix<double,3,4> & m = f->shell->camToWorld.matrix3x4();
				
				std::cout << "Hessian Points Size: " << f->pointHessiansMarginalized.size() << std::endl; 
				
				if (f->pointHessiansMarginalized.size() > 0)
				{
					std::string filename = "trunk/vision/" + CERNOutputWrapper::long2string((long)(f->shell->timestamp)) + ".txt";
					std::cout << "CERNOutputWrapper timestamp = " << (long)f->shell->timestamp << std::endl;
					_timestampVector->push_back((long)f->shell->timestamp);
					std::cout << "Timestamp pushed " << std::endl;
					_pccounter = 0;
					_savepts.open(filename, std::ios::app);
					
					for(PointHessian* p : f->pointHessiansMarginalized)
					{
						float depth = 1.0f / (p->idepth); // try idepth_scaled also 
						
						Eigen::Vector4d camPoint;
						
						camPoint(0) = (p->u * fxi + cxi) * depth;
						camPoint(1) = (p->v * fyi + cyi) * depth;
						camPoint(2) = depth*(1 + 2*fxi * (rand()/(float)RAND_MAX-0.5f)); // look at this...
						camPoint(3) = 1.0f;
						
						/*printf("OUT: Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
						p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian), p->numGoodResiduals );*/
						//std::cout << "camPoint=" << camPoint << std::endl;

						Eigen::Vector3d worldPoints = m*camPoint;
						
						_savepts << worldPoints(0) << " " << worldPoints(1) << " " << worldPoints(2) << "\n";
						_pccounter++;	
								
						
					}
					
					std::cout << "CERN: Saved " << _pccounter << " points to " << filename << std::endl;
					_savepts.close();
				}
				else{
				}
				
			}
	}

		
		
		void CERNOutputWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
		{
	
			trajectory.open("trunk/xyz.txt", std::ios::app);
				
			if(_live)
			{
				
				printf("CERN: Current Frame %d (time %li, internal ID %d)\n",
					frame->incoming_id,
					(long)frame->timestamp,
					frame->id);
				
				trajectory << (long)frame->timestamp <<
					" " << frame->camToWorld.translation()[0] <<
					" " << frame->camToWorld.translation()[1] <<
					" " << frame->camToWorld.translation()[2] <<
					" " << frame->camToWorld.rotationMatrix()(0,0) <<
					" " << frame->camToWorld.rotationMatrix()(0,1) <<
					" " << frame->camToWorld.rotationMatrix()(0,2) <<
					" " << frame->camToWorld.rotationMatrix()(1,0) <<
					" " << frame->camToWorld.rotationMatrix()(1,1) <<
					" " << frame->camToWorld.rotationMatrix()(1,2) <<
					" " << frame->camToWorld.rotationMatrix()(2,0) <<
					" " << frame->camToWorld.rotationMatrix()(2,1) <<
					" " << frame->camToWorld.rotationMatrix()(2,2) << "\n";
				
				std::cout << "X = " << frame->camToWorld.translation()[0] << " Y = " << frame->camToWorld.translation()[1] << " Z = " << frame->camToWorld.translation()[2] << std::endl;
			}
			else
			{
			
				printf("CERN: Current Frame %d (time %f, internal ID %d)\n",
						frame->incoming_id,
						frame->timestamp,
						frame->id);
				
				std::cout << "X = " << frame->camToWorld.translation()[0] << " Y = " << frame->camToWorld.translation()[1] << " Z = " << frame->camToWorld.translation()[2] << std::endl;
				
				trajectory << (long)frame->timestamp <<
					" " << frame->camToWorld.translation()[0] <<
					" " << frame->camToWorld.translation()[1] <<
					" " << frame->camToWorld.translation()[2] <<
					" " << frame->camToWorld.rotationMatrix()(0,0) <<
					" " << frame->camToWorld.rotationMatrix()(0,1) <<
					" " << frame->camToWorld.rotationMatrix()(0,2) <<
					" " << frame->camToWorld.rotationMatrix()(1,0) <<
					" " << frame->camToWorld.rotationMatrix()(1,1) <<
					" " << frame->camToWorld.rotationMatrix()(1,2) <<
					" " << frame->camToWorld.rotationMatrix()(2,0) <<
					" " << frame->camToWorld.rotationMatrix()(2,1) <<
					" " << frame->camToWorld.rotationMatrix()(2,2) << "\n";
			}
			
			trajectory.close();
		
		}


		void CERNOutputWrapper::pushLiveFrame(FrameHessian* image)
		{
	// can be used to get the raw image / intensity pyramid.
		}

		void CERNOutputWrapper::pushDepthImage(MinimalImageB3* image)
		{
			// can be used to get the raw image with depth overlay.
		}
		
		bool CERNOutputWrapper::needPushDepthImage()
		{
			return false;
		}

		void CERNOutputWrapper::pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF )
		{
			/*printf("CERN: Predicted depth for KF %d (id %d, time %f, internal frame-ID %d). CameraToWorld:\n",
				KF->frameID,
				KF->shell->incoming_id,
				KF->shell->timestamp,
				KF->shell->id);
			std::cout << KF->shell->camToWorld.translation() << "\n";*/
			
			
			}
	
		
	}

}
