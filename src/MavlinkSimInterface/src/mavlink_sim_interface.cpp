/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 * Author: Roman Fedorenko <frontwise@gmail.com>
 */

#include "MavlinkSimInterface/mavlink_sim_interface.hpp"
#include <iostream>
#include <cstring>
#include <cmath>
#include <mavlink/common/mavlink.h>
#include <poll.h>
#include <netinet/tcp.h>
#include <unistd.h>

int MavlinkSimInterface::Init(int portOffset) {
    std::cout << "Waiting for connection from PX4..." << std::endl;

    memset((char *) &simulatorMavlinkAddr_, 0, sizeof(simulatorMavlinkAddr_));
    memset((char *) &px4MavlinkAddr_, 0, sizeof(px4MavlinkAddr_));
    simulatorMavlinkAddr_.sin_family = AF_INET;
    simulatorMavlinkAddr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    simulatorMavlinkAddr_.sin_port = htons(PORT_BASE + portOffset);

    if ((listenMavlinkSock_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Error: Creating TCP socket failed: " << strerror(errno) << std::endl;
        return -1;
    }

    int yes = 1;
    int result = setsockopt(listenMavlinkSock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (result != 0) {
        std::cerr << "Error: setsockopt failed: " << strerror(errno) << std::endl;
    }

    struct linger nolinger;
    nolinger.l_onoff = 1;
    nolinger.l_linger = 0;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
    if (result != 0) {
        std::cerr << "Error: setsockopt failed: " << strerror(errno) << std::endl;
    }

    int socket_reuse = 1;
    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) {
        std::cerr << "Error: setsockopt failed: " << strerror(errno) << std::endl;
    }

    result = setsockopt(listenMavlinkSock_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
    if (result != 0) {
        std::cerr << "Error: setsockopt failed: " << strerror(errno) << std::endl;
    }

    if (bind(listenMavlinkSock_, (struct sockaddr *)&simulatorMavlinkAddr_, sizeof(simulatorMavlinkAddr_)) < 0) {
        std::cerr << "Error: bind failed: " << strerror(errno) << std::endl;
    }

    result = listen(listenMavlinkSock_, 5);
    if (result < 0) {
        std::cerr << "Error: listen failed: " << strerror(errno) << std::endl;
    }

    unsigned int px4_addr_len = sizeof(px4MavlinkAddr_);
    while (true) {
        px4MavlinkSock_ = accept(listenMavlinkSock_,
                                  (struct sockaddr *)&px4MavlinkAddr_,
                                  &px4_addr_len);
        if (px4MavlinkSock_ < 0) {
            std::cerr << "Error: accept failed: " << strerror(errno) << std::endl;
        } else {
            std::cout << "PX4 Connected." << std::endl;
            break;
        }
    }

    _inited = true;

    return result;
}

int MavlinkSimInterface::Clean() {
    close(px4MavlinkSock_);
    close(listenMavlinkSock_);
    return 0;
}

int MavlinkSimInterface::SendHilSensor(uint64_t time_usec,
                                       float gpsAltitude,
                                       Eigen::Vector3d magFrd,
                                       Eigen::Vector3d accFrd,
                                       Eigen::Vector3d gyroFrd,
                                       float staticPressure,
                                       float staticTemperature,
                                       float diffPressureHPa) {
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = time_usec;
    sensor_msg.id = 0;

    sensor_msg.xacc = accFrd[0];
    sensor_msg.yacc = accFrd[1];
    sensor_msg.zacc = accFrd[2];
    sensor_msg.xgyro = gyroFrd[0];
    sensor_msg.ygyro = gyroFrd[1];
    sensor_msg.zgyro = gyroFrd[2];
    sensor_msg.fields_updated = SENS_ACCEL | SENS_GYRO;

    if (time_usec - lastMagTimeUsec_ > MAG_PERIOD_US) {
        sensor_msg.xmag = magFrd[0];
        sensor_msg.ymag = magFrd[1];
        sensor_msg.zmag = magFrd[2];
        sensor_msg.fields_updated |= SENS_MAG;
        lastMagTimeUsec_ = time_usec;
    }

    if (time_usec - lastBaroTimeUsec_ > BARO_PERIOD_US) {
        sensor_msg.temperature = staticTemperature;
        sensor_msg.abs_pressure = staticPressure;
        sensor_msg.pressure_alt = gpsAltitude;
        // sensor_msg.pressure_alt += baroAltNoise_ * normalDistribution_(randomGenerator_);
        sensor_msg.diff_pressure = diffPressureHPa;
        sensor_msg.fields_updated |= SENS_BARO | SENS_DIFF_PRESS;
        lastBaroTimeUsec_ = time_usec;
    }

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

    if (send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen) {
        return -1;
    }
    return 0;
}

int MavlinkSimInterface::SendHilGps(uint64_t time_usec,
                                    Eigen::Vector3d linearVelNed,
                                    Eigen::Vector3d gpsPosition) {
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = time_usec;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = gpsPosition.x() * 1e7;
    hil_gps_msg.lon = gpsPosition.y() * 1e7;
    hil_gps_msg.alt = gpsPosition.z() * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vn = linearVelNed.x() * 100;
    hil_gps_msg.ve = linearVelNed.y() * 100;
    hil_gps_msg.vd = linearVelNed.z() * 100;
    hil_gps_msg.vel = std::sqrt(hil_gps_msg.vn * hil_gps_msg.vn + hil_gps_msg.ve * hil_gps_msg.ve);

    double cog = -std::atan2(hil_gps_msg.vn, hil_gps_msg.ve) * 180 / 3.141592654 + 90;
    if (cog < 0) {
        cog += 360;
    }
    hil_gps_msg.cog = cog * 100;
    hil_gps_msg.satellites_visible = 10;

    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);
    if (packetlen == 0 || send(px4MavlinkSock_, buffer, packetlen, 0) != packetlen) {
        return -1;
    }
    return 0;
}

int MavlinkSimInterface::Receive(bool blocking, bool &armed, std::array<double, 16>& command) {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    struct pollfd fds[1] = {};
    fds[0].fd = px4MavlinkSock_;
    fds[0].events = POLLIN;

    int p = poll(&fds[0], 1, (blocking ? -1 : 2));
    if (p < 0) {
        return -1;
    }
    
    if (p == 0) {
        return 0;
    }

    if (fds[0].revents & POLLIN) {
        unsigned int slen = sizeof(px4MavlinkAddr_);
        unsigned int len = recvfrom(px4MavlinkSock_,
                                    buffer,
                                    sizeof(buffer),
                                    0,
                                    (struct sockaddr *)&px4MavlinkAddr_,
                                    &slen);
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                    mavlink_hil_actuator_controls_t controls;
                    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                    armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    if (armed) {
                        for (size_t idx = 0; idx < 16; idx++) {
                            command[idx] = controls.controls[idx];
                        }
                    } else {
                        command.fill(0.0);
                    }

                    return 1;
                }
            }
        }
    
        return 0;
    }

    return -1;
}
