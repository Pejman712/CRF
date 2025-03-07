/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2023
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <sstream>

#include "CANopenDrivers/CoEMaster/CoEMaster.hpp"

namespace crf::devices::canopendrivers {

CoEMaster::CoEMaster(
    const std::string& ifname,
    const uint& numberOfSlaves,
    const std::chrono::microseconds& cycleTime,
    const nlohmann::json& slavesConfig,
    const int& sizeOfIOMap,
    std::shared_ptr<ISOEMAPI> soemApi):
    ethercatdrivers::EtherCATMaster(ifname, numberOfSlaves, cycleTime, sizeOfIOMap, soemApi),
    logger_("CoEMaster") {
    logger_->debug("CTor");

    if (slavesConfig.is_array()) {
        slavesConfig_ = slavesConfig.get<std::vector<nlohmann::json>>();
        return;
    }

    if (!slavesConfig.at("SlaveID").is_array()) {
        slavesConfig_.push_back(slavesConfig);
        return;
    }

    for (uint64_t i = 0;  i < slavesConfig.at("SlaveID").size(); i++) {
        nlohmann::json pdo;
        pdo = slavesConfig;
        pdo["SlaveID"] = slavesConfig.at("SlaveID")[i];
        slavesConfig_.push_back(pdo);
    }
}

CoEMaster::~CoEMaster() {
    logger_->debug("DTor");
}

bool CoEMaster::onPreOperational() {
    logger_->info("Configuring {} slaves in the network", numberOfSlaves_);

    /*
     * Since JSON does not accept hexadecimal representation we decided to put them as
     * strings in the JSON and convert here to hex rather than writting the dec number
     * If we move to another type of config like YAML that accepts hex this won't be a
     * problem.
     * (jplayang)
     */
    for (auto&& slave : slavesConfig_) {
        uint16_t slaveID = slave["SlaveID"].get<uint16_t>();
        logger_->info("Configuring slave {}", slaveID);

        // SDO Configuration
        if (slave.contains("SDOConfiguration")) {
            for (auto&& sdo : slave.at("SDOConfiguration")) {
                logger_->debug("{}", sdo);
                uint16_t idx;
                std::stringstream ssidx;
                ssidx << std::hex << sdo.at("Idx").get<std::string>();
                ssidx >> idx;

                uint16_t subidx;
                std::stringstream sssubidx;
                sssubidx << std::hex << sdo.at("SubIdx").get<std::string>();
                sssubidx >> subidx;

                logger_->info("Configuring SDO Idx: 0x{:04X}, Subidx: 0x{:02X}", idx, subidx);

                if (sdo.at("Type") == "UINT8") {
                    if (!writeSDOAndParse<uint8_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "UINT16") {
                    if (!writeSDOAndParse<uint16_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "UINT32") {
                    if (!writeSDOAndParse<uint32_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "UINT64") {
                    if (!writeSDOAndParse<uint64_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "INT8") {
                    if (!writeSDOAndParse<int8_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "INT16") {
                    if (!writeSDOAndParse<int16_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "INT32") {
                    if (!writeSDOAndParse<int32_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "INT64") {
                    if (!writeSDOAndParse<int64_t>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "FLOAT") {
                    if (!writeSDOAndParse<float>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "DOUBLE") {
                    if (!writeSDOAndParse<double>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                } else if (sdo.at("Type") == "STRING") {
                    if (!writeSDOAndParse<std::string>(
                        slaveID, idx, subidx, sdo.at("Value").get<std::string>())) {
                        logger_->error("Could not set default SDO");
                        return false;
                    }
                }
            }
        }

        if (slave.contains("PDOConfiguration")) {
            // PDO Configuration
            // RxPDO
            logger_->info("Configuring RxPDO slave ID {}", slaveID);

            // Disable RxPDOAssign by writing zero
            if (!writeSDO<uint8_t>(slaveID, CiA301::RxPDOAssign, Subindex::SUB0, 0x00)) {
                logger_->error("RxPDO register could not be disabled");
                return false;
            }

            // Fill RxPDO mapping
            uint8_t subindexAssign = 0x00;
            for (auto&& RxPDO : slave.at("PDOConfiguration").at("RxPDOAssign")) {
                subindexAssign++;
                uint16_t rxpdoAssign;
                std::stringstream ss;
                ss << std::hex << RxPDO.at("Idx").get<std::string>();
                ss >> rxpdoAssign;

                logger_->info("Assinging RxPDO to 0x{:04X}, sub 0x{:02X}",
                    rxpdoAssign, subindexAssign);

                if (!writeSDO<uint16_t>(slaveID, CiA301::RxPDOAssign, subindexAssign, rxpdoAssign)) {  // NOLINT
                    logger_->error("RxPDO register could not be assigned to 0x{:04X}",
                        rxpdoAssign);
                    return false;
                }

                if (!RxPDO.contains("Map")) continue;

                if (!writeSDO<uint8_t>(slaveID, rxpdoAssign, Subindex::SUB0, 0x00)) {
                    logger_->error("RxPDO register could not be disabled");
                    return false;
                }

                uint8_t subindexMap = 0x01;
                for (auto&& regis : RxPDO.at("Map")) {
                    uint32_t idx_subIdx_bitl;
                    std::stringstream ssRegis;
                    ssRegis << std::hex << regis.at("Idx").get<std::string>();
                    ssRegis << std::hex << regis.at("SubIdx").get<std::string>();
                    ssRegis << std::hex << regis.at("BitLength").get<std::string>();
                    ssRegis >> idx_subIdx_bitl;

                    logger_->info("Mapping PDO 0x{:08X} into idx: 0x{:04X}, subidx: 0x{:02X}",
                        idx_subIdx_bitl, rxpdoAssign, subindexMap);
                    if (!writeSDO<uint32_t>(slaveID, rxpdoAssign, subindexMap, idx_subIdx_bitl)) {
                        logger_->error("RxPDO register could not be written");
                        return false;
                    }
                    subindexMap++;
                }

                subindexMap--;

                if (!writeSDO<uint8_t>(slaveID, rxpdoAssign, Subindex::SUB0, subindexMap)) {
                    logger_->error("RxPDO register could not be enabled");
                    return false;
                }
            }

            // Enable RxPDOAssign by writing the number of mapped elements
            if (!writeSDO<uint8_t>(slaveID, CiA301::RxPDOAssign, Subindex::SUB0, subindexAssign)) {
                logger_->error("RxPDO register could not be enabled, {}", subindexAssign);
                return false;
            }

            // TxPDO
            logger_->info("Configuring TxPDO slave ID {}", slaveID);

            // Disable TxPDOAssign by writing zero
            if (!writeSDO<uint8_t>(slaveID, CiA301::TxPDOAssign, Subindex::SUB0, 0x00)) {
                logger_->error("TxPDO register could not be disabled");
                return false;
            }

            // Fill TxPDO mapping
            subindexAssign = 0x00;
            for (auto&& TxPDO : slave["PDOConfiguration"]["TxPDOAssign"]) {
                subindexAssign++;
                uint16_t txpdoAssign;
                std::stringstream ss;
                ss << std::hex << TxPDO["Idx"].get<std::string>();
                ss >> txpdoAssign;

                logger_->info("Assinging TxPDO to 0x{:04X}, sub 0x{:02X}",
                    txpdoAssign, subindexAssign);

                if (!writeSDO<uint16_t>(slaveID, CiA301::TxPDOAssign, subindexAssign, txpdoAssign)) {  // NOLINT
                    logger_->error("TxPDO register could not be assigned to 0x{:04X}",
                        txpdoAssign);
                    return false;
                }

                if (!TxPDO.contains("Map")) continue;

                if (!writeSDO<uint8_t>(slaveID, txpdoAssign, Subindex::SUB0, 0x00)) {
                    logger_->error("TxPDO register could not be disabled");
                    return false;
                }

                uint8_t subindexMap = 0x01;
                for (auto&& regis : TxPDO["Map"]) {
                    uint32_t idx_subIdx_bitl;
                    std::stringstream ssRegis;
                    ssRegis << std::hex << regis["Idx"].get<std::string>();
                    ssRegis << std::hex << regis["SubIdx"].get<std::string>();
                    ssRegis << std::hex << regis["BitLength"].get<std::string>();
                    ssRegis >> idx_subIdx_bitl;

                    logger_->info("Mapping PDO 0x{:08X} into idx: 0x{:04X}, subidx: 0x{:02X}",
                        idx_subIdx_bitl, txpdoAssign, subindexMap);
                    if (!writeSDO<uint32_t>(slaveID, txpdoAssign, subindexMap, idx_subIdx_bitl)) {
                        logger_->error("TxPDO register could not be written");
                        return false;
                    }
                    subindexMap++;
                }

                subindexMap--;

                if (!writeSDO<uint8_t>(slaveID, txpdoAssign, Subindex::SUB0, subindexMap)) {
                    logger_->error("TxPDO register could not be enabled");
                    return false;
                }
            }

            // Enable TxPDOAssign by writing the number of mapped elements
            if (!writeSDO<uint8_t>(slaveID, CiA301::TxPDOAssign, Subindex::SUB0, subindexAssign)) {
                logger_->error("TxPDO register could not be enabled, {}", subindexAssign);
                return false;
            }
            logger_->info("PDOS for slave ID {} done", slaveID);
        }
    }

    logger_->info("PDOs configured");
    return true;
}

}  // namespace crf::devices::canopendrivers
