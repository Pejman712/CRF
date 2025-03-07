#pragma once

namespace crf {
namespace sensors {

	typedef struct _ObjectInfo_t{
		uint32_t StorageId;
		uint16_t ObjectFormat;
		uint16_t ProtectionStatus;
		uint32_t ObjectCompressedSize;
	} ObjectInfo_t;

	typedef struct _LiveViewInfo_t{
		uint32_t Offset_to_LiveView_Image;
		uint32_t LiveVew_Image_Size;
	} LiveViewInfo_t;
	
	
	class SOCCptpstring {
	public :
		SOCCptpstring(const char* c){
			std::string ascii(c);
			bytes_size = 1+(ascii.size()+1)*2;
			bytes = new uint8_t[bytes_size];
			bytes[0] = ascii.size();
			for(int i=0; i<ascii.size()+1; i++){
				bytes[1+i*2] = c[i];
				bytes[1+i*2+1] = 0;
			}
		}
		~SOCCptpstring(){
			delete [] bytes;
		}
		uint8_t* bytes;
		uint16_t bytes_size;
	};

}
}