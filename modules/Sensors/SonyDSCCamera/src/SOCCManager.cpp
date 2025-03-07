#include <SonyDSCCamera/SOCCManager.hpp>

using namespace com::sony::imaging::remote;

crf::sensors::SOCCManager::SOCCManager ( com::sony::imaging::remote::socc_ptp& ptp ) : ptp(ptp) {

}

bool crf::sensors::SOCCManager::connect() {
	int ret = ptp.connect();

	if (ret != 0) {
		return false;
	}
	
	return true;	
}

bool crf::sensors::SOCCManager::disconnect() {
	int ret = ptp.disconnect();
	
	if (ret != 0) {
		return false;
	}
	
	return true;
}

bool crf::sensors::SOCCManager::openSession (uint32_t session_id) {
	int ret;
	uint32_t params[1];
	Container response;
	
	do {
		params[0] = session_id;
		manager_mutex.lock();
		ret = ptp.send(0x1002, params, 1, response, NULL, 0);
		manager_mutex.unlock();
		session_id++;
	} while(ret != 0);
	
	if (response.code != (uint16_t)0x2001) {		
		return false;	
	}
	
	return true;
}

bool crf::sensors::SOCCManager::closeSession() {
	int ret;
	Container response;
	
	manager_mutex.lock();
	ret = ptp.send(0x1003, NULL, 0, response, NULL, 0);
	manager_mutex.unlock();

	if (ret !=0) { 
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		return false;	
	}
	
	return true;
}

bool crf::sensors::SOCCManager::getObjectInfo ( uint32_t handle, crf::sensors::ObjectInfo_t* object_info ) {
	int ret;
	ObjectInfo_t* data = NULL;
	uint32_t size = 0;
	uint32_t params[1];
	Container response;
	params[0] = handle;

	manager_mutex.lock();
	ret = ptp.receive(0x1008, params, 1, response, (void**)&data, size);
	manager_mutex.unlock();

	if(object_info != NULL){
		*object_info = *data;
	}
	
	if (ret !=0) { 
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	ptp.dispose_data((void**)&data);
	return true;
}


bool crf::sensors::SOCCManager::getObject ( uint32_t handle, void** object_data, uint32_t* compressed_size ) {
	int ret;
	void* data = NULL;
	FILE* fpo = NULL;
	uint32_t size = 0;
	uint32_t params[1];
	Container response;
	params[0] = handle;
	
	manager_mutex.lock();
	ret = ptp.receive(0x1009, params, 1, response, (void**)&data, size);
	manager_mutex.unlock();

	if (ret !=0) { 
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		ptp.dispose_data((void**)&data);
		return false;	
	}

	if(object_data != NULL && compressed_size != NULL){
		*compressed_size = size;
		*object_data = malloc(size);
		memcpy(*object_data, data, size);
	}
	ptp.dispose_data((void**)&data);
	
	return true;
}


bool crf::sensors::SOCCManager::SDIOGetAllExtDevicePropInfo ( SDIDevicePropInfoDatasetArray** array ) {
	int ret;
	void* data = NULL;
	uint32_t size = 0;
	uint32_t params[0];
	Container response;

	manager_mutex.lock();
	ret = ptp.receive(0x96F6, params, 0, response, (void**)&data, size);
	manager_mutex.unlock();

	if (ret !=0) { 
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		ptp.dispose_data((void**)&data);
		return false;	
	}
	

	*array = new SDIDevicePropInfoDatasetArray(data);
	ptp.dispose_data((void**)&data);
	
	return true;
}

bool crf::sensors::SOCCManager::SDIOSetExtDevicePropValue ( uint16_t code, crf::sensors::SOCCptpstring ptpstring ) {
	int ret;
	uint32_t params[1];
	Container response;
	params[0] = code;

	manager_mutex.lock();
	ret = ptp.send(0x96FA, params, 1, response, ptpstring.bytes, ptpstring.bytes_size);
	manager_mutex.unlock();

	if (ret !=0) { 
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		return false;	
	}
	
	return true;
}

bool crf::sensors::SOCCManager::SDIOGetExtDeviceInfo ( uint16_t initiator_version, uint16_t* actual_initiator_version ) {
	bool escape = false;
	int ret;
	uint16_t *version;
	uint32_t size = 0;
	uint32_t params[1];
	Container response;
	params[0] = (uint32_t)initiator_version;

	manager_mutex.lock();
	ret  = ptp.receive(0x96FD, params, 1, response, (void**)&version, size);
	manager_mutex.unlock();

	if(actual_initiator_version != NULL){
		*actual_initiator_version = *version;
	}
	ptp.dispose_data((void**)&version);
	
	if (ret !=0) { 
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		return false;	
	}
	
	return true;
}


bool crf::sensors::SOCCManager::waitForInitiatorVersion ( uint16_t expect, int retry_count ) {
	uint16_t actual;
	
	while(retry_count>0){
		actual = ~expect;
		retry_count--;
		SDIOGetExtDeviceInfo((uint32_t)expect, &actual);
		if(expect == actual){
			break;
		}
	}
	
	if (expect != actual) {
		return false;		
	}
	return true;
}


bool crf::sensors::SOCCManager::SDIOConnect ( uint32_t phase_type, uint32_t keycode1, uint32_t keycode2 ) {
	int ret;
	uint64_t* data;
	uint32_t params[3];
	Container response;
	uint32_t size;
	params[0] = phase_type;
	params[1] = keycode1;
	params[2] = keycode2;

	manager_mutex.lock();
	ret = ptp.receive(0x96FE, params, 3, response, (void**)&data, size);
	manager_mutex.unlock();
	
	if (ret !=0) { 
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	if (response.code != (uint16_t)0x2001) {
		ptp.dispose_data((void**)&data);
		return false;	
	}
	
	ptp.dispose_data((void**)&data);
	
	return true;

}


bool crf::sensors::SOCCManager::waitEvent ( uint16_t code ) {
	int ret;
	bool escape = false;
	while(escape == false){
		Container event;
		ret = ptp.wait_event(event);
		if(ret == SOCC_ERROR_USB_TIMEOUT){
			continue;
		}
		if(event.code == code){
			break;
		}
		
		if (ret != 0){
			return false;
		}
	};
	
	return true;
}


bool crf::sensors::SOCCManager::dropEvent ( uint16_t code ) {
	Container event;
	int ret = ptp.wait_event(event);
	
	if (ret == SOCC_ERROR_USB_TIMEOUT) {
		return false;
	}
	
	if (event.code == code) {
		return true;
	}
	
	return false;
}

void crf::sensors::SOCCManager::milisleep ( uint16_t msec ) {
	struct timespec req;
	req.tv_nsec = (msec * 1000 * 1000) % 1000000000;
	req.tv_sec = msec / 1000;
	nanosleep(&req, NULL);
}



