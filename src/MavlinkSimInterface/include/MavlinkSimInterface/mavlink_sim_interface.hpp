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

#ifndef MAVLINK_SIM_INTERFACE_MAVLINK_SIM_INTERFACE_HPP
#define MAVLINK_SIM_INTERFACE_MAVLINK_SIM_INTERFACE_HPP

#include <netinet/in.h>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

class MavlinkSimInterface {
public:
    MavlinkSimInterface() {};

    /**
     * @brief Init connection with PX4 using TCP
     */
    int Init(int portOffset);

    /**
     * @brief Close mavlink sockets
     */
    int Clean();

    /**
     * @brief Send hil_sensor (#107) and hil_gps (#113) to PX4 via mavlink
     */
    int SendHilSensor(uint64_t time_usec,
                      float gpsAltitude,
                      Eigen::Vector3d mag_frd,
                      Eigen::Vector3d acc_frd,
                      Eigen::Vector3d gyro_frd,
                      float staticTemperature,
                      float staticPressure,
                      float diffPressureHPa);
    int SendHilGps(uint64_t time_usec,
                   Eigen::Vector3d vel_ned,
                   Eigen::Vector3d pose_geodetic);

    /**
     * @brief Receive hil_actuator_controls (#93) from PX4 via mavlink
     * @param blocking - input
     * @param armed - output
     * @param command - output
     */
    int Receive(bool blocking, bool &armed, std::array<double, 16>& command);

    static inline void process(const Eigen::Vector3d& vel_ned,
                               const Eigen::Vector3d& geodetic_position,
                               const Eigen::Vector3d& magnetic_field_frd,
                               const Eigen::Vector3d& accFrd,
                               const Eigen::Vector3d& gyroFrd,
                               float temperatureCelcius,
                               float staticPressureHpa,
                               float diffPressureHPa,
                               std::vector<double>& actuators,
                               uint8_t& is_armed)
    {
        static MavlinkSimInterface flight_stack;
        if (!flight_stack._inited) {
            flight_stack.Init(0);
            return;
        }

        double crnt_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now().time_since_epoch()).count();
        size_t time_usec = crnt_time * 1000000;

        // Recv
        bool isArmed;
        std::array<double, 16> actuators_raw;
        if(flight_stack.Receive(false, isArmed, actuators_raw) == 1){
            for (size_t actuator_idx = 0; actuator_idx < 16; actuator_idx++) {
                actuators[actuator_idx] = actuators_raw[actuator_idx];
            }
            is_armed = isArmed ? 1 : 2;
        }

        // Send imu
        int status = flight_stack.SendHilSensor(time_usec,
                                                geodetic_position.z(),
                                                magnetic_field_frd,
                                                accFrd,
                                                gyroFrd,
                                                staticPressureHpa,
                                                temperatureCelcius,
                                                diffPressureHPa);
        if (status == 0) {
        }

        // Send gps
        static size_t last_gps_time_usec = 0;
        size_t GPS_PERIOD_US = 1e6 / 10;
        if (time_usec >= last_gps_time_usec + GPS_PERIOD_US){
            last_gps_time_usec = time_usec;

            if(flight_stack.SendHilGps(time_usec, vel_ned, geodetic_position) == 0){
            }
        }
    }


private:
    static const uint64_t SENS_ACCEL       = 0b111;
    static const uint64_t SENS_GYRO        = 0b111000;
    static const uint64_t SENS_MAG         = 0b111000000;
    static const uint64_t SENS_BARO        = 0b1101000000000;
    static const uint64_t SENS_DIFF_PRESS  = 0b10000000000;

    static constexpr uint64_t MAG_PERIOD_US = 1e6 / 100;
    static constexpr uint64_t BARO_PERIOD_US = 1e6 / 50;
    uint64_t lastMagTimeUsec_ = 0;
    uint64_t lastBaroTimeUsec_ = 0;

    const int PORT_BASE = 4560;
    struct sockaddr_in px4MavlinkAddr_;
    struct sockaddr_in simulatorMavlinkAddr_;
    int listenMavlinkSock_;
    int px4MavlinkSock_;

    bool _inited{false};
};

#endif  // MAVLINK_SIM_INTERFACE_MAVLINK_SIM_INTERFACE_HPP
