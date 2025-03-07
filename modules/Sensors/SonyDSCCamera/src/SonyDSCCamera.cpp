#include <SonyDSCCamera/SonyDSCCamera.hpp>
#include <SonyDSCCamera/libsoccptp/parser.h>

using namespace com::sony::imaging::remote;

crf::sensors::SonyDSCCamera::SonyDSCCamera(std::shared_ptr<IPC> input_commands_ipc, std::shared_ptr<IPC> live_output_ipc, std::shared_ptr<IPC> command_output_ipc) :
	ptp(0,0),
	manager(ptp)
{
	this->input_commands_ipc = input_commands_ipc;
	this->live_output_ipc = live_output_ipc;
	this->command_output_ipc = command_output_ipc;

	_stopThreads = false;
}

bool crf::sensors::SonyDSCCamera::open() {	
	if (!manager.connect()) {
		return false;
	}
	
	printf("Connection successful\n");
	bool session_started = false;

	for (int i=0; i<2; i++) {
		if (manager.openSession()) {
			session_started = true;
			break;
		} else {
			manager.closeSession();
		}
	}

	if (!session_started){
		printf("Failed starting the session\n");
		return false;
	}
	
	printf("Session correctly started\n");	
	
	if (!manager.SDIOConnect(0x000001)) {
		printf("Failed SDIOConnect(0x000001)\n");
		return false;
	}
	
	if (!manager.SDIOConnect(0x000002)) {
		printf("Failed SDIOConnect(0x000002)\n");
		return false;
	}
	
	if (!manager.waitForInitiatorVersion((uint16_t)0x00C8)) {
		printf("Failed waitForInitiatorVersion((uint16_t)0x00C8)\n");
		return false;
	}
	
	if (!manager.SDIOConnect(0x000003)) {
		printf("Failed SDIOConnect(0x000003)\n");
		return false;
	}
	
	manager.waitForIsEnable(0xD6B1, 0x01);
	
	if (!manager.SDIOSetExtDevicePropValue(0xD6B1,SOCCptpstring("20150801T150000+0900"))) {
		printf("Failed manager.SDIOSetExtDevicePropValue(0xD6B1,SOCCptpstring(\"20150801T150000+0900\"))\n");
		return false;		
	}
	
	// Put camera to Still Rec Mode
	manager.waitForIsEnable(0xD6E2, 0x01);
    manager.SDIOSetExtDevicePropValue(0xD6E2, (uint8_t)0x03);
    manager.waitForCurrentValue(0xD6E2, (uint8_t)0x03);

	// Put SaveMedia to Host device
    manager.SDIOSetExtDevicePropValue(0xD6CF, (uint16_t)0x0001);
    manager.waitForCurrentValue(0xD6CF, (uint16_t)0x0001);
    
	// Wait that Live View is enabled
    manager.waitForCurrentValue(0xD6DE, (uint8_t)0x01);

	manager.SDIOSetExtDevicePropValue(0xD6CC, (uint16_t)0x8000);
    manager.waitForCurrentValue(0xD6CC, (uint16_t)0x8000);
	
	manager.milisleep(500);

	_isOpen = true;

	live_grabber_thread = std::thread(&SonyDSCCamera::live_grabber, this);
	camera_info_grabber_thread = std::thread(&SonyDSCCamera::camera_info_grabber, this);

	camera_info_grabber_thread.detach();

	setLiveViewResolution(LiveViewResolutions::LOW);
	setCompressionSetting(Compressions::FineCompression);
	
	return true;
}

bool crf::sensors::SonyDSCCamera::close() {	
	if (_isOpen) {
		_stopThreads = true;
		live_grabber_thread.join();

		if (!manager.closeSession()) {
			return false;
		};

		if (!manager.disconnect()) {
			return false;			
		}
	};

	
	_isOpen = false;
	return true;
}

void crf::sensors::SonyDSCCamera::live_grabber() {
	while(!_stopThreads) {
		std::string jpeg_bytes = captureLiveView();
		if (jpeg_bytes.length() > 0) {
			Packets::JpegImagePacket packet;
			packet.setJpegBytes(jpeg_bytes, false);
			live_output_ipc->write(packet.serialize(), packet.getHeader());
		}

		usleep(66000);
	}
}

void crf::sensors::SonyDSCCamera::camera_info_grabber() {
	std::string buffer;
	Packets::PacketHeader header;

	while(!_stopThreads) {
		input_commands_ipc->read(buffer, header);
		
		if (header.type == Packets::CAMERA_SETTING_PACKET) {
			Packets::CameraSettingPacket setting;
			setting.deserialize(buffer);
			
			if (setting.liveImageResolutionWidth == 1920) {
				setLiveViewResolution(LiveViewResolutions::HIGH);
			} else if (setting.liveImageResolutionWidth == 1024) {
				setLiveViewResolution(LiveViewResolutions::MEDIUM);
			} else if (setting.liveImageResolutionWidth == 640) {
				setLiveViewResolution(LiveViewResolutions::LOW);
			}

			if (setting.zoom>= 0) {
				setZoomPosition(setting.zoom);
			}
			
			/*setISO(setting.ISO);
			setShutterSpeed(setting.shutterSpeed);*/
		}
	}
}

void crf::sensors::SonyDSCCamera::checkShootingInformation() {	
	while(1){
        uint16_t shooting_information;
        uint16_t* CurrentValue = manager.getCurrentValue<uint16_t>((uint16_t)0xD6CC);

        if(CurrentValue == NULL){
            continue;
        }

        shooting_information = *CurrentValue;
        delete CurrentValue;

        if((shooting_information & 0x8000) && (shooting_information & 0x7FFF)){
            printf("Shooting information(0xD6C6),CurrentValue=%04x\n",shooting_information);
            break;
        }
    }
}

std::string crf::sensors::SonyDSCCamera::capture() {	
	_captureMutex.lock();
	manager.SDIOSetExtDevicePropValue(0xD6E2, (uint8_t)0x02);
    manager.waitForCurrentValue(0xD6E2, (uint8_t)0x02);

	while(!manager.SDIOControlDevice(0xD61D, (uint16_t)0x0002)) {};
	manager.waitForCurrentValue(0xD61D, (uint16_t)0x0002);

	// Button S2 Down
	while(!manager.SDIOControlDevice(0xD617, (uint16_t)0x0002)) {};
	manager.milisleep(100);

	// Button S2 Up
	while(!manager.SDIOControlDevice(0xD617, (uint16_t)0x0001)) {};
	manager.milisleep(10);

	// Button S1 Up
	while(!manager.SDIOControlDevice(0xD61D, (uint16_t)0x0001)) {};

	ObjectInfo_t object_info;
	do {
		manager.getObjectInfo(0xFFFFC001, &object_info);
	} while(object_info.ObjectCompressedSize == 0);


	void* object_data;
	uint32_t compressed_size;

	if (!manager.getObject(0xFFFFC001, &object_data, &compressed_size)) {
		manager.SDIOSetExtDevicePropValue(0xD6E2, (uint8_t)0x03);
    	manager.waitForCurrentValue(0xD6E2, (uint8_t)0x03);
		_captureMutex.unlock();
		return std::string();
	};

	std::string jpeg_bytes ((char*)object_data, compressed_size);
	free(object_data);	

	manager.SDIOSetExtDevicePropValue(0xD6E2, (uint8_t)0x03);
    manager.waitForCurrentValue(0xD6E2, (uint8_t)0x03);
	_captureMutex.unlock();

	return jpeg_bytes;
}

std::string crf::sensors::SonyDSCCamera::captureLiveView() {
	_captureMutex.lock();
	void* object_data;
	uint32_t compressed_size;

	bool ret = manager.getObject(0xFFFFC002, &object_data, &compressed_size);

	if (!ret) {
		_captureMutex.unlock();
		return std::string();
	}

	int header;
	int length;
	memcpy(&header, object_data,4);
	memcpy(&length, object_data+4, 4);

	std::string jpeg_bytes ((char*)object_data+header, compressed_size-header);
	free(object_data);
	_captureMutex.unlock();
	
	return jpeg_bytes;
}

bool crf::sensors::SonyDSCCamera::setLiveViewResolution(crf::sensors::SonyDSCCamera::LiveViewResolutions resolution) {
	uint8_t resbyte = 0x00;

	switch(resolution) {
		case LiveViewResolutions::LOW:
			resbyte = 0x00;
			break;
		case LiveViewResolutions::MEDIUM:
			resbyte = 0x01;
			break;
		case LiveViewResolutions::HIGH:
			resbyte = 0x02;
			break;	
	}

	_captureMutex.lock();
	if (!manager.SDIOSetExtDevicePropValue(0xD6AA, resbyte)) {
		printf("Error while changing resolution\n");
		_captureMutex.unlock();
		return false;		
	}

    manager.waitForCurrentValue(0xD6AA, resbyte);
	_captureMutex.unlock();

	return true;
}

crf::sensors::SonyDSCCamera::LiveViewResolutions crf::sensors::SonyDSCCamera::getLiveViewResolution() {
	uint8_t* resbyte = manager.getCurrentValue<uint8_t>((uint16_t)0xD6AA);

	if (*resbyte == 0x00) {
		return LiveViewResolutions::LOW;
	} else if (*resbyte == 0x01) {
		return LiveViewResolutions::MEDIUM;
	} else {
		return LiveViewResolutions::HIGH;
	}
}

bool crf::sensors::SonyDSCCamera::setAspectRatio(AspectRatios aspectRatio) {
	uint8_t resbyte = 0x03;

	switch(aspectRatio) {
		case AspectRatios::AR3_2:
			resbyte = 0x01;
			break;
		case AspectRatios::AR16_9:
			resbyte = 0x02;
			break;
		case AspectRatios::AR4_3:
			resbyte = 0x03;
			break;	
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6B3, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6B3, resbyte);
	return true;
}

crf::sensors::SonyDSCCamera::AspectRatios crf::sensors::SonyDSCCamera::getAspectRatio() {
	uint8_t* resbyte = manager.getCurrentValue<uint8_t>((uint16_t)0xD6B3);

	if (*resbyte == 0x01) {
		return AspectRatios::AR3_2;
	} else if (*resbyte == 0x02) {
		return AspectRatios::AR16_9;
	} else {
		return AspectRatios::AR4_3;
	}
}

bool crf::sensors::SonyDSCCamera::setCompressionSetting(Compressions compression) {
	uint8_t resbyte = 0x02;

	switch(compression) {
		case Compressions::StandardCompression:
			resbyte = 0x02;
			break;
		case Compressions::FineCompression:
			resbyte = 0x03;
			break;	
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6B9, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6B9, resbyte);
	return true;
}

crf::sensors::SonyDSCCamera::Compressions crf::sensors::SonyDSCCamera::getCompressionSetting() {
	uint8_t* resbyte = manager.getCurrentValue<uint8_t>((uint16_t)0xD6B9);

	if (*resbyte == 0x02) {
		return Compressions::StandardCompression;
	} else {
		return Compressions::FineCompression;
	}
}

bool crf::sensors::SonyDSCCamera::setColorTemperature(uint16_t temperature) {
	uint16_t resbyte;

	if (temperature < 0x09C4) {
		resbyte = 0x09C4;
	} else if (temperature > 0x26AC) {
		resbyte = 0x026AC;
	} else {
		resbyte = temperature;
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6F0, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6F0, resbyte);
	return true;
}

uint16_t crf::sensors::SonyDSCCamera::getColorTemperature() {
	uint16_t* resbyte = manager.getCurrentValue<uint16_t>((uint16_t)0xD6F0);

	return *resbyte;
}

bool crf::sensors::SonyDSCCamera::setExposureCompensation(int16_t exposure) {
	int16_t resbyte;

	if (exposure >= 0x07D0) {
		resbyte = 0x07D0;
	} else if (exposure >= 0x06A4) {
		resbyte = 0x06A4;
	} else if (exposure >= 0x0514) {
		resbyte = 0x0514;
	} else if (exposure >= 0x03E8) {
		resbyte = 0x03E8;
	} else if (exposure >= 0x02BC) {
		resbyte = 0x02BC;
	} else if (exposure >= 0x012C) {
		resbyte = 0x012C;
	} else if (exposure >= 0x0000) {
		resbyte =  0x0000;
	} else if (exposure >= 0xFED4) {
		resbyte = 0xFED4;
	} else if (exposure >= 0xFD44) {
		resbyte = 0xFD44;
	} else if (exposure >= 0xFC18) {
		resbyte = 0xFC18;
	} else if (exposure >= 0xFAEC) {
		resbyte = 0xFAEC;
	} else if (exposure >= 0xF95C) {
		resbyte = 0xF95C;
	} else {
		resbyte = 0xF830;
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6C3, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6C3, resbyte);
	return true;
}

int16_t crf::sensors::SonyDSCCamera::getExposureCompensation() {
	int16_t* resbyte = manager.getCurrentValue<int16_t>((uint16_t)0xD6C3);

	return *resbyte;
}

bool crf::sensors::SonyDSCCamera::setExposureMode(crf::sensors::SonyDSCCamera::ExposureModes mode) {
	uint16_t resbyte = 0x03;

	switch(mode) {
		case ExposureModes::Manual:
			resbyte = 0x0001;
			break;
		case ExposureModes::ProgramAuto:
			resbyte = 0x0002;
			break;
		case ExposureModes::AperturePriority:
			resbyte = 0x0003;
			break;
		case ExposureModes::ShutterPriority:
			resbyte = 0x0004;
			break;	
		case ExposureModes::IntelligentAuto:
			resbyte = 0x8000;
			break;	
		case ExposureModes::SuperiorAuto:
			resbyte = 0x8001;
			break;		
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6CC, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6CC, resbyte);
	return true;
}

crf::sensors::SonyDSCCamera::ExposureModes crf::sensors::SonyDSCCamera::getExposureMode() {
	uint16_t* resbyte = manager.getCurrentValue<uint16_t>((uint16_t)0xD6CC);

	switch(*resbyte) {
		case 0x0001:
			return ExposureModes::Manual;
			break;
		case 0x0002:
			return ExposureModes::ProgramAuto;
			break;
		case 0x0003:
			return ExposureModes::AperturePriority;
			break;
		case 0x0004:
			return ExposureModes::ShutterPriority;
			break;
		case 0x8000:
			return ExposureModes::IntelligentAuto;
			break;
		case 0x8001:
			return ExposureModes::SuperiorAuto;
			break;
	}
}

bool crf::sensors::SonyDSCCamera::setFNumber(float value) {
	uint16_t resbyte;

	if (value >= 8) {
		resbyte = 0x0320;
	} else if (value >= 7.1) {
		resbyte = 0x02C6;
	} else if (value >= 6.3) {
		resbyte = 0x0276;
	} else if (value >= 5.6) {
		resbyte = 0x0230;
	} else if (value >= 5) {
		resbyte = 0x01F4;
	} else if (value >= 4.5) {
		resbyte = 0x01C2;
	} else if (value >= 4) {
		resbyte = 0x0190;
	} else {
		resbyte = 0x015E;
	}

	if (!manager.SDIOSetExtDevicePropValue(0xD6C5, resbyte)) {
		return false;		
	}

    manager.waitForCurrentValue(0xD6C5, resbyte);
	return true;
}

float crf::sensors::SonyDSCCamera::getFNumber() {
	uint16_t* resbyte = manager.getCurrentValue<uint16_t>((uint16_t)0xD6C5);

	switch(*resbyte) {
		case 0x015E:
			return 3.5f;
			break;
		case 0x0190:
			return 4.0f;
			break;
		case 0x01C2:
			return 4.5f;
			break;
		case 0x0230:
			return 5.0f;
			break;
		case 0x0276:
			return 6.3f;
			break;
		case 0x02C6:
			return 7.1f;
			break;
		case 0x0320:
			return 8.0f;
			break;
	}
}

crf::sensors::SonyDSCCamera::FocusIndications crf::sensors::SonyDSCCamera::getFocusIndication() {
	uint8_t* resbyte = manager.getCurrentValue<uint8_t>((uint16_t)0xD6EC);

	switch(*resbyte) {
		case 0x01:
			return FocusIndications::AFUnlock;
			break;
		case 0x02:
			return FocusIndications::AFLock;
			break;
		case 0x03: 
			return FocusIndications::AFLockWarning;
			break;
		case 0x05:
			return FocusIndications::InFocusing;
			break;
		case 0x06:
			return FocusIndications::FocusedInContinuosMode;
			break;
	}
}

bool crf::sensors::SonyDSCCamera::setFocusMode(crf::sensors::SonyDSCCamera::FocusModes mode) {

}

crf::sensors::SonyDSCCamera::FocusModes crf::sensors::SonyDSCCamera::getFocusMode() {
	uint16_t* resbyte = manager.getCurrentValue<uint16_t>((uint16_t)0xD6CB);

	switch(*resbyte) {
		case 0x0001:
			return FocusModes::ManualFocus;
			break;
		case 0x0002:
			return FocusModes::AutoFocus;
			break;
		case 0x8004:
			return FocusModes::AutoFocus;
			break;
	}
}

bool crf::sensors::SonyDSCCamera::setImageSize(crf::sensors::SonyDSCCamera::ImageSizes) {

}

crf::sensors::SonyDSCCamera::ImageSizes crf::sensors::SonyDSCCamera::getImageSize() {
	uint8_t* resbyte = manager.getCurrentValue<uint8_t>((uint16_t)0xD6B7);

	switch(*resbyte) {
		case 0x01:
			return ImageSizes::L;
			break;
		case 0x02:
			return ImageSizes::M;
			break;
		case 0x03: 
			return ImageSizes::S;
			break;
		case 0x19:
			return ImageSizes::VGA;
			break;
	}
}

bool crf::sensors::SonyDSCCamera::setISO(int iso) {

}

int crf::sensors::SonyDSCCamera::getISO() {
	uint32_t* resbyte = manager.getCurrentValue<uint32_t>((uint16_t)0xD6F2);

	return (int)(*resbyte);
}

bool crf::sensors::SonyDSCCamera::setShutterSpeed(float speed) {

}

float crf::sensors::SonyDSCCamera::getShutterSpeed() {
	uint32_t* resbyte = manager.getCurrentValue<uint32_t>((uint16_t)0xD6EA);

	switch (*resbyte) {
		case 0x012C000A:
			return 30.0f;
			break;
		case 0x00FA000A:
			return 25.0f;
			break;
		case 0x00C8000A:
			return 20.0f;
			break;
		case 0x0096000A:
			return 15.0f;
			break;
		case 0x0082000A:
			return 13.0f;
			break;
		case 0x0064000A:
			return 10.0f;
			break;
		case 0x0050000A:
			return 8.0f;
			break;
		default:
			return 0.0f;
			break;
	}
}

bool crf::sensors::SonyDSCCamera::setWhiteBalance(crf::sensors::SonyDSCCamera::WhiteBalanceSettings) {

}

crf::sensors::SonyDSCCamera::WhiteBalanceSettings crf::sensors::SonyDSCCamera::getWhiteBalance() {

}

bool crf::sensors::SonyDSCCamera::setZoomPosition(int value) {
	int actualPosition = getZoomPosition();
	value /= 100;
	if (value == actualPosition) {
		return true;
	}

	int direction = actualPosition > value ? -1 : 1;

	if (direction == 1) {
		manager.SDIOControlDevice(0xD63C, (uint16_t)0x0002);
		manager.milisleep(100);
		while(getZoomPosition() < value) {};
		manager.SDIOControlDevice(0xD63C, (uint16_t)0x0001);
		manager.milisleep(100);
	} else {
		manager.SDIOControlDevice(0xD63E, (uint16_t)0x0002);
		manager.waitForCurrentValue(0xD63E, (uint16_t)0x0002);
		manager.milisleep(100);
		while(getZoomPosition() > value) {};
		manager.SDIOControlDevice(0xD63E, (uint16_t)0x0001);
		manager.waitForCurrentValue(0xD63E, (uint16_t)0x0001);
		manager.milisleep(100);
	}

	return true;
}

int crf::sensors::SonyDSCCamera::getZoomPosition() {
	SDIDevicePropInfoDatasetArray* array = NULL;
	manager.SDIOGetAllExtDevicePropInfo(&array);

	int zoom = (int)((DataTypeArray<uint8_t>*)array->get(0xD6BF))->CurrentValues[0];

	delete(array);

	return zoom;
}