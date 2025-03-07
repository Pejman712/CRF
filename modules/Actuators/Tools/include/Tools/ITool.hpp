#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <string>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace devices {
namespace tools {

enum ToolValueTypes { BOOL, INT, FLOAT };

class ITool : public utility::commoninterfaces::IInitializable {
 public:
    ~ITool() override = default;
    /**
     * @brief Initializes the tool
     * If required, the tool is powered, calibrated and brought to an initial position
     * @return true if initialization was succesful
     * @return false otherwise
     */
    bool initialize() override = 0;

    /**
     * @brief Deinitializes the tool
     * If required, the power is removed and the tool is brought to a safe position
     * @return true if deinitialization was succesful
     * @return false otherwise
     */
    bool deinitialize() override = 0;

    /**
     * @brief Set the boolean value of the tool
     * The name of the value that needs to be set must be provided
     * @param name the name of the value to set
     * @param value the value to set
     * @return true if the value was correctly set
     * @return false if the provided name did not exist, if the provided value type is not correct or if it was not possible to set the value of the tool
     */
    virtual bool setValue(const std::string& name, bool value) = 0;
    /**
     * @brief Set the integer value of the tool
     * The name of the value that needs to be set must be provided
     * @param name the name of the value to set
     * @param value the value to set
     * @return true if the value was correctly set
     * @return false if the provided name did not exist, if the provided value type is not correct or if it was not possible to set the value of the tool
     */
    virtual bool setValue(const std::string& name, int value) = 0;
    /**
     * @brief Set the float value of the tool
     * The name of the value that needs to be set must be provided
     * @param name the name of the value to set
     * @param value the value to set
     * @return true if the value was correctly set
     * @return false if the provided name did not exist, if the provided value type is not correct or if it was not possible to set the value of the tool
     */
    virtual bool setValue(const std::string& name, float value) = 0;

    /**
     * @brief Get the value of the tool
     * The name of the value that needs to be get must be provided
     * @param name the name of the value to get
     * @return boost::any if the value was correctly get
     * @return boost::none if the provided name did not exist or if it was not possible to get the value of the tool
     */
    virtual boost::optional<boost::any> getValue(const std::string& name) const = 0;

    /**
     * @brief Get the type of the value of the tool
     * The name of the value type that needs to be get must be provided
     * @param name the name of the value type to get
     * @return ToolValueTypes the type of the value
     * @return boost::none if the provided name did not exist
     */
    virtual boost::optional<ToolValueTypes> getValueType(const std::string& name) = 0;

    /**
     * @brief Get the list of value names available for the tool
     * @return std::vector<std::string> the list of value names
     */
    virtual std::vector<std::string> getValueNames() = 0;
};

}  // namespace tools
}  // namespace devices
}  // namespace crf
