#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>

#include <IPC/IPC.hpp>
#include <Camera/CameraPackets.hpp>

#include <SonyDSCCamera/libsoccptp/socc_ptp.h>
#include <SonyDSCCamera/SOCCManager.hpp>

namespace crf {
namespace sensors {
	
class SonyDSCCamera {
	com::sony::imaging::remote::socc_ptp ptp;
	SOCCManager manager;

	std::shared_ptr<IPC> input_commands_ipc;
	std::shared_ptr<IPC> live_output_ipc; 
	std::shared_ptr<IPC> command_output_ipc;

	bool _isOpen;

	volatile bool _stopThreads;

	std::thread live_grabber_thread;
	void live_grabber();

	std::thread camera_info_grabber_thread;
	void camera_info_grabber();

	std::thread commands_receiver_thread;
	void commands_receiver();

	std::mutex _captureMutex;
	
public:
	SonyDSCCamera(std::shared_ptr<IPC> input_commands_ipc, std::shared_ptr<IPC> live_output_ipc, std::shared_ptr<IPC> command_output_ipc);
	
	bool open();
	bool close();

	std::string capture();
	std::string captureLiveView();

	void checkShootingInformation();

	enum LiveViewResolutions { LOW, MEDIUM, HIGH };
	bool setLiveViewResolution(LiveViewResolutions resolution);
	LiveViewResolutions getLiveViewResolution();

	enum AspectRatios { AR3_2, AR16_9, AR4_3 };
	bool setAspectRatio(AspectRatios aspectRatio);
	AspectRatios getAspectRatio();

	bool setColorTemperature(uint16_t temperature);
	uint16_t getColorTemperature();

	enum Compressions { StandardCompression, FineCompression };
	bool setCompressionSetting(Compressions compression);
	Compressions getCompressionSetting();

	bool setExposureCompensation(int16_t exposure);
	int16_t getExposureCompensation();

	enum ExposureModes { Manual, ProgramAuto, AperturePriority, ShutterPriority, IntelligentAuto, SuperiorAuto };
	bool setExposureMode(ExposureModes mode);
	ExposureModes getExposureMode();

	bool setFNumber(float value);
	float getFNumber();

	enum FocusIndications { AFUnlock, AFLock, AFLockWarning, InFocusing, FocusedInContinuosMode };
	FocusIndications getFocusIndication();

	enum FocusModes { ManualFocus, AutoFocus };
	bool setFocusMode(FocusModes mode);
	FocusModes getFocusMode();

	enum ImageSizes { L, M, S, VGA };
	bool setImageSize(ImageSizes);
	ImageSizes getImageSize();

	bool setISO(int);
	int getISO();

	bool setShutterSpeed(float);
	float getShutterSpeed();

	enum WhiteBalanceSettings { Auto, Daylight, Incandescent, CoolWhite, DayWhite, FluorDaylight, Cloudy, ColorTempFilter };
	bool setWhiteBalance(WhiteBalanceSettings);
	WhiteBalanceSettings getWhiteBalance();
	
	bool setZoomPosition(int);
	int getZoomPosition();
};

}
}