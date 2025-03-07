/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Krzysztof Szczurek CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <boost/any.hpp>

#include "CommonInterfaces/IInitializable.hpp"
#include "SiemensPLC/RegisterType.hpp"

namespace crf {
namespace devices {
namespace siemensplc {

class ISiemensPLC : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~ISiemensPLC() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /*
     * @brief
     * @return
     * @return
     */
    virtual bool isConnected() = 0;
    /*
     * @brief The method writes a register to a DB casting the value according to the register
     *        type. For the Bool type the parameter bitNumber has to be specified, for other types
     *        this parameter is not used
     * @return True if it succeeded.
     * @return False if it failed.
     */
    virtual bool writeRegister(crf::devices::siemensplc::RegisterType registerType,
        const boost::any& value, unsigned int dbNumber, unsigned int registerOffset,
        unsigned int bitNumber) = 0;
    /*
     * @brief The method reads a register from a DB casting the value according to the register
     *        type. For the Bool type the parameter bitNumber has to be specified, for other types
     *        this parameter is not used
     * @return The value in the requested type.
     * @return Empty boost::any if it failed.
     */
    virtual boost::any readRegister(crf::devices::siemensplc::RegisterType registerType,
        unsigned int dbNumber, unsigned int registerOffset, unsigned int bitNumber) = 0;
    /*
     * @brief The method reads an entire the bytes contained in a db for a certain length. For
     *        conversion to current values, SiemensPLCTypeConvert must be used and the structure
     *        of the db must be known.
     * @return The return boost::string is in the DB bit format. 
     */
    virtual std::string readDB(const unsigned int dbNumber, const size_t length,
        const unsigned int registerOffset) = 0;
};

}  // namespace siemensplc
}  // namespace devices
}  // namespace crf
