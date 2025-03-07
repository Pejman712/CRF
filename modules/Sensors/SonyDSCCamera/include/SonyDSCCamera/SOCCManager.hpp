#pragma once

#include <iostream>
#include <mutex>
#include <string.h>
#include <time.h>

#include <SonyDSCCamera/libsoccptp/parser.h>

#include <SonyDSCCamera/libsoccptp/socc_ptp.h>
#include <SonyDSCCamera/SonyDSCCameraTypes.hpp>

namespace crf {
namespace sensors{

using namespace com::sony::imaging::remote;

class SOCCManager {
	com::sony::imaging::remote::socc_ptp ptp;

	std::mutex manager_mutex;	
public:
	SOCCManager(com::sony::imaging::remote::socc_ptp& ptp);
	
	bool connect();
	bool disconnect();
	
	bool openSession(uint32_t session_id=1);
	bool closeSession();
	
	bool getObjectInfo(uint32_t handle, ObjectInfo_t* object_info=NULL);
	bool getObject(uint32_t handle, void** object_data=NULL, uint32_t* compressed_size=NULL);
	
	bool SDIOGetAllExtDevicePropInfo(com::sony::imaging::remote::SDIDevicePropInfoDatasetArray** array);
	
	template <typename T>
    bool waitForCurrentValue(uint16_t code, T expect, int count=1000);
	template <typename SDIControl_value_t>
    bool SDIOControlDevice(uint16_t code, SDIControl_value_t value);
	template <typename T>
    bool SDIOSetExtDevicePropValue(uint16_t code, T value);
	bool SDIOSetExtDevicePropValue(uint16_t code, SOCCptpstring ptpstring);
	bool SDIOGetExtDeviceInfo(uint16_t initiator_version=0x00C8, uint16_t* actual_initiator_version = NULL);
	bool waitForInitiatorVersion(uint16_t expect = 0x00C8, int retry_count = 1000);
	bool SDIOConnect(uint32_t phase_type, uint32_t keycode1=0x0000DA01, uint32_t keycode2=0x0000DA01);
	bool waitEvent(uint16_t code);
	bool dropEvent(uint16_t code);

	template <typename T>
    com::sony::imaging::remote::SDIDevicePropInfoDataset* waitForIsEnable(uint16_t code, T expect, int count=1000);
	template <typename T>
    T* getCurrentValue(uint16_t code, int count=1000);
	
	void milisleep(uint16_t msec);
};

template <typename T>
SDIDevicePropInfoDataset* crf::sensors::SOCCManager::waitForIsEnable ( uint16_t code, T expect, int count ) {
	SDIDevicePropInfoDataset* dataset = NULL;
	while(count>0){
		count--;
		SDIDevicePropInfoDatasetArray* array = NULL;
		SDIOGetAllExtDevicePropInfo(&array);
		if(array == NULL){
			continue;
		}
		dataset = array->get(code);
		if(dataset==NULL){
			delete array;
			continue;
		}
		T isEnable = dataset->IsEnable;
		delete array;
		if(isEnable == expect){
			break;
		}
	}

	return dataset;
}

template <typename T>
T* crf::sensors::SOCCManager::getCurrentValue ( uint16_t code, int count ) {
	 T* CurrentValue = NULL;
	DataTypeInteger<T>* dataset = NULL;
	while(count>0){
		count--;
		SDIDevicePropInfoDatasetArray* array = NULL;
		SDIOGetAllExtDevicePropInfo(&array);
		if(array == NULL){
			continue;
		}
		dataset = (DataTypeInteger<T>*)array->get(code);
		if(dataset==NULL){
			delete array;
			continue;
		}

		std::string c;
		dataset->toString(c);

		CurrentValue = new T();
		*CurrentValue = dataset->CurrentValue;
		delete array;
		break;
	}
	
	return CurrentValue;
}

template <typename T>
bool crf::sensors::SOCCManager::waitForCurrentValue ( uint16_t code, T expect, int count ) {
	DataTypeInteger<T>* dataset = NULL;
	while(count>0){
		count--;
		SDIDevicePropInfoDatasetArray* array = NULL;
		SDIOGetAllExtDevicePropInfo(&array);
		if(array == NULL){
			continue;
		}
		dataset = (DataTypeInteger<T>*)array->get(code);
		if(dataset==NULL){
			delete array;
			continue;
		}
		T CurrentValue = dataset->CurrentValue;
		delete array;
		if(CurrentValue == expect){
			break;
		}
	}
	
	if(dataset==NULL){
		return false;
	}
	
	return true;
}


template <typename SDIControl_value_t>
bool crf::sensors::SOCCManager::SDIOControlDevice ( uint16_t code, SDIControl_value_t value ) {
	int ret;
	uint32_t params[1];
	Container response;
	params[0] = code;
	ret = ptp.send(0x96F8, params, 1, response, &value, sizeof(value));
	
	if (ret !=0) { 
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		return false;	
	}
	
	return true;
}


template <typename T>
bool crf::sensors::SOCCManager::SDIOSetExtDevicePropValue ( uint16_t code, T value ) {
	int ret;
	uint32_t params[1];
	Container response;
	params[0] = code;
	ret = ptp.send(0x96FA, params, 1, response, &value, sizeof(value));
	
	if (ret !=0) { 
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		return false;	
	}
	
	return true;
}

}
}