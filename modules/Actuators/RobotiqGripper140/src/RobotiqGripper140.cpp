/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <RobotiqGripper140/RobotiqGripper140.hpp>

RobotiqGripper140::RobotiqGripper140(char* dev_name) : Gripper(0.0, 0.14) {
    device_name = dev_name;

    ctx = modbus_new_rtu(device_name, 115200, 'N', 8, 1);
    if (ctx == NULL) {
        printf("Could not connect to Gripper (new rtu) \n");
        perror("Gripper");
        return;
    }

    modbus_set_slave(ctx, 9);
    if (modbus_connect(ctx) == -1) {
        modbus_free(ctx);
        ctx = NULL;
        printf("Could not connect to Gripper (modbus_connect) \n");
        return;
    }

    deactivate_gripper();
    while (gripper_status != 3) {
        get_gripper_status();
        activate_gripper();
        usleep(20000);
    }

    target_position_char = 0;
    target_speed_char = 100;
    target_force_char = 50;

    control_mode = POSITION_GRIPPER_CONTROL;
    gripper_thread = std::thread(&RobotiqGripper140::run, this);
    gripper_thread.detach();
}

bool RobotiqGripper140::deactivate_gripper() {
    uint16_t msg[3];
    msg[0] = 0x0;
    msg[1] = 0x0;
    msg[2] = 0x0;

    if (modbus_write_registers(ctx, 0x03E8, 3, msg) == -1) {
        perror("Activate gripper");
        return false;
    }
    return true;
}

bool RobotiqGripper140::activate_gripper() {
    uint16_t msg[3];

    msg[0] = 0x0100;
    msg[1]= 0x0;
    msg[2] = 0x0;

    if (modbus_write_registers(ctx, 0x03E8, 3, msg) == -1) {
      perror("Activate gripper");
      return false;
    }
    return get_gripper_status();
}

bool RobotiqGripper140::is_connected() {
    if (ctx == NULL) {
        return false;
    }
    return true;
}

bool RobotiqGripper140::get_gripper_status() {
    if (!is_connected()) {
        return false;
    }

    uint16_t msg[3];
    if (modbus_read_registers(ctx, 0x07D0, 3, msg) == -1) {
        perror("get_gripper_status()");
        return false;
    }

    unsigned char bytes[6];
    memcpy(bytes, msg, 6);

    is_active = bytes[1] & 0x1;
    gripper_status = (bytes[1] & 0b00110000) >> 4;
    object_detection = (bytes[1] & 0b11000000) >> 6;
    actual_position_char = bytes[5];
    actual_finger_current_char = bytes[4];
    return true;
}

bool RobotiqGripper140::set_gripper_status() {
    if (!is_connected()) {
        return false;
    }

    uint16_t msg[3];
    msg[0] = 0x0900;
    msg[1]= (target_position_char) + (0xFF <<8);
    msg[2] = (target_speed_char << 8) + target_force_char;

    if (modbus_write_registers(ctx, 0x03E8, 3, msg) == -1) {
        perror("Activate gripper");
        return false;
    }
    return true;
}

void RobotiqGripper140::run() {
    while (true) {
        if (gripper_status == 3) {
            set_gripper_status();
        } else if (gripper_status == 1) {
            activate_gripper();
        }

        get_gripper_status();
        actual_position = (actual_position_char/-1557.14) + 0.14;

        if (control_mode == POSITION_GRIPPER_CONTROL) {
            float value = -1557.14*(target_position - 0.14);
            if (value > 255) {
                value = 255;
            } else if (value < 0) {
                value = 0;
            }

            target_position_char = (unsigned char)value;
            target_speed_char = 100;
            target_force_char = 50;
        } else if (control_mode == VELOCITY_GRIPPER_CONTROL) {
            int target_speed_int = (unsigned int)(fabs(target_velocity)*400);
            if (target_speed_int > 240) {
                target_speed_char = 240;
            } else if (target_speed_int < 15) {
                target_speed_char = 15;
            } else {
                target_speed_char = target_speed_int;
            }

            float value = -1557.14 * ((actual_position + target_velocity*0.3) - 0.14);
            if (value > 255) {
                value = 255;
            } else if (value < 0) {
                value = 0;
            }

            target_position_char = (unsigned char)value;
            if (is_grasping()) {
                printf("Grasping\n");
            }
            if ((is_grasping())&&(target_velocity < 0.0)) {
                if (target_force_char > 244) {
                    target_force_char = 254;
                } else {
                    target_force_char += 10;
                }
            } else {
                target_force_char = 50;
            }
        }
        usleep(50000);
    }
}

float RobotiqGripper140::getFingerCurrent() {
    return static_cast<float>(actual_finger_current_char);
}

int RobotiqGripper140::is_grasping() {
    if ((object_detection == 1) || (object_detection == 2)) {
        return 1;
    }
    return 0;
}
