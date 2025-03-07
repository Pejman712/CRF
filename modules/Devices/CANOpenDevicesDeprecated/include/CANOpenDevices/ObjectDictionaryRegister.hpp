/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 * 
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <string>

namespace crf {
namespace devices {
namespace canopendevices {

class ObjectDictionaryRegister {
 public:
    /*
     * @brief Construct a new Object Dictionary Register object
     * @param name Name of the register
     * @param size Size of the register in bytes. Allowed size are 1, 2 or 4 bytes
     * @param index Index of the register
     * @param subindex Subindex of the register. Default value is 0
     */
    ObjectDictionaryRegister(const std::string& name, uint8_t size, uint16_t index,
      uint8_t subindex = 0);
    ObjectDictionaryRegister(const ObjectDictionaryRegister& other) = default;
    ObjectDictionaryRegister(ObjectDictionaryRegister&& other) = default;
    ObjectDictionaryRegister() = delete;
    ~ObjectDictionaryRegister() = default;

    /*
     * @brief Get the name register.
     * @return std::string the name of the register.
     */
    std::string getName() const;
    /*
     * @brief Get the size of the register in bytes.
     * @return uint8_t the size of the register in bytes.
     */
    uint8_t getSize() const;
    /*
     * @brief Get the index of the register.
     * @return uint16_t the index of the register.
     */
    uint16_t getIndex() const;
    /*
     * @brief Get the subindex of the register.
     * @return uint8_t the subindex of the register.
     */
    uint8_t getSubindex() const;
    /*
     * @brief Get the timestamp of the last update of the value of the register.
     * @return std::chrono::time_point<std::chrono::high_resolution_clock> timestamp of the last
     *         update of the register.
     */
    std::chrono::time_point<std::chrono::high_resolution_clock> getLastUpdate();
    /*
     * @brief Set the value of the register.
     * @tparam T type of the register. Allowed types are uint8_t, uint16_t, uint32_t, int8_t,
     *         int16_t and int32_t.
     * @param value the value to set.
     * @return true if the value was correctly set.
     * @return false otherwise.
     */
    template<typename T>
    bool setValue(T value);
    /*
     * @brief Set the value of the register from a byte string. The byte string must have the same
     *        size of the register.
     * @return true if the value was correctly set.
     * @return false otherwise.
     */
    bool setValue(const std::string&);
    /*
     * @brief Get the value of the register.
     * @tparam T type of the register. Allowed types are uint8_t, uint16_t, uint32_t, int8_t,
     *         int16_t and int32_t.
     * @return T the value of the register.
     */
    template<typename T>
    T getValue() const;

    ObjectDictionaryRegister& operator=(const ObjectDictionaryRegister&) = default;

 private:
    std::string name_;
    uint8_t size_;
    uint16_t index_;
    uint8_t subindex_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastUpdate_;

    std::string value_;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
