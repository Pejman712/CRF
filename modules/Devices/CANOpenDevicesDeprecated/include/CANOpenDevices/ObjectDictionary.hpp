/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <sys/types.h>
#include <utility>
#include <vector>

#include "CANOpenDevices/ObjectDictionaryRegister.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class ObjectDictionary {
 public:
    /*
     * @brief Construct a new Object Dictionary object 
     * @param configurationDictionary the path to the object dictionary configuration file
     */
    explicit ObjectDictionary(const std::string& configurationDictionary);
    ObjectDictionary(const ObjectDictionary& other) = delete;
    ObjectDictionary(ObjectDictionary&& other) = delete;
    ObjectDictionary() = delete;
    ~ObjectDictionary() = default;

    /*
     * @brief Adds a register to hte object dictionary. It fails if a register with the same name
     *        or index and subindex was already added.
     * @return true the register was correctly added
     * @return false otherwise
     */
    bool addRegister(const ObjectDictionaryRegister&);
    /*
     * @brief Adds a PDO mapping to the object dictionary. It fails if the total size of the passed
     *        registers is bigger than 8 bytes.
     * @param pdo the PDO code for the mapping.
     * @param registers the registers in the PDO.
     * @return true if the mapping was correctly done.
     * @return false otherwise.
     */
    bool addPDOMapping(uint8_t pdo, const std::vector<ObjectDictionaryRegister>& registers);
    /*
     * @brief can_frame handling for the object dictionary. It checks if it is a SDO, PDO or NMT
     *        message and it deals it with accordingly.
     * @param cframe the can frame.
     * @return true the data was correctly handled.
     * @return false otherwise.
     */
    bool setData(const can_frame& cframe);
    /*
     * @brief Get the reference to a register object using the index and the subindex
     * @param registerIndex the index of the register
     * @param subRegisterIndex the subindex of the register
     * @return std::nullopt if the register didn't exist
     * @return ObjectDictionaryRegister& the reference to the requested register
     */
    boost::optional<ObjectDictionaryRegister&> getRegister(uint16_t registerIndex,
        uint8_t subRegisterIndex = 0);
    /*
     * @brief Get the reference to a register object using its name
     * @param name the name of the register
     * @return std::nullopt if the register didn't exist
     * @return ObjectDictionaryRegister& the reference to the requested register
     */
    boost::optional<ObjectDictionaryRegister&> getRegister(const std::string& name);
    /*
     * @brief Waits for the SDO response on the requested register
     * @param reg the register on which to wait for an SDO response
     * @param timeout waiting time before returning
     * @return std::nullopt the timeout was expired
     * @return std::pair<ObjectDictionaryRegister&, bool>> the reference to the register on which
     *         the SDO operation was requested and the result of the operation (true if sucessful,
     *         false otherwise)
     */
    boost::optional<std::pair<ObjectDictionaryRegister&, bool> > waitForSdoResponse(
        const ObjectDictionaryRegister& reg, const std::chrono::milliseconds& timeout);
    /*
     * @brief Get all the registers in the object dictionary
     * @return std::vector<ObjectDictionaryRegister> 
     */
    std::vector<ObjectDictionaryRegister> getRegistersVector();
    /*
     * @brief Returns the timestamp of the last NMT heartbeat that was received and the NMT state
     *        of the device.
     * @return std::pair<uint8_t, std::chrono::high_resolution_clock::time_point> 
     */
    std::pair<uint8_t, std::chrono::high_resolution_clock::time_point> getNMTState();

 private:
    utility::logger::EventLogger logger_;

    std::map<uint32_t, ObjectDictionaryRegister> registers_;
    std::map<std::string, uint32_t> registersByName_;
    std::map<uint8_t, std::vector<ObjectDictionaryRegister> > pdoMappings_;
    std::pair<uint8_t, std::chrono::high_resolution_clock::time_point> nmtState_;

    bool parseConfiguration(const std::string& configurationDictionary);
    void handlersInizialization();
    uint8_t getFunctionCode(const can_frame&);
    uint16_t getIndex(const can_frame&);
    uint8_t getSubIndex(const can_frame&);

    std::map<uint8_t, std::function<bool(ObjectDictionary*, const can_frame&)>>
        canFrameHandlerMap_;
    bool sdoHandler(const can_frame&);
    bool pdoHandler(const can_frame&);
    bool nmtHandler(const can_frame&);

    std::atomic<bool> sdoCvFlag_;
    std::vector<std::pair<ObjectDictionaryRegister&, bool> > sdoResponseResults_;
    std::mutex sdoMutex_;
    std::condition_variable sdoConditionVariable_;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
