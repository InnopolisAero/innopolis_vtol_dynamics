/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ARDUPILOT_JSON_INTERFACE_ARDUPILOT_JSON_INTERFACE_HPP
#define ARDUPILOT_JSON_INTERFACE_ARDUPILOT_JSON_INTERFACE_HPP

#include <chrono>
#include <array>
#include <cmath>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <Eigen/Dense>
#include "libAP_JSON.cpp"

class ArdupilotJsonInterface
{
public:
    ArdupilotJsonInterface() : _inited(false), _ap_online(false) {}

    int Init(const std::string &ip, int port)
    {
        if (_libAP.InitSockets(ip.c_str(), port)) {
            std::cout << "[ArdupilotJsonInterface] Connected to ArduPilot SITL at "
                      << ip << ":" << port << std::endl;
            _inited = true;
            return 0;
        }
        std::cerr << "[ArdupilotJsonInterface] ERROR: Failed to init sockets." << std::endl;
        return -1;
    }

    int Clean()
    {
        _inited = false;
        return 0;
    }

    int Receive(bool /*blocking*/, bool &armed, std::array<double, 16> &command)
    {
        if (!_inited) {
            std::cerr << "[ArdupilotJsonInterface] ERROR: Not initialized." << std::endl;
            return -1;
        }
        uint16_t servo_out[MAX_SERVO_CHANNELS];
        bool got_packet = _libAP.ReceiveServoPacket(servo_out);
        if (got_packet) {
            bool all_zero = true;
            for (size_t i = 0; i < MAX_SERVO_CHANNELS; i++) {
                if (servo_out[i] != 0) {
                    all_zero = false;
                    break;
                }
            }
            armed = !all_zero;
            for (size_t i = 0; i < 16; i++) {
                command[i] = (i < MAX_SERVO_CHANNELS) ? servo_out[i] : 0.0;
            }
            _ap_online = true;
            return 1;
        }
        return 0;
    }

    int SendSimData(double time_now_sec,
                    const Eigen::Vector3d &velNED,
                    const Eigen::Vector3d &posGeodetic,
                    const Eigen::Quaterniond &attitudeFrdNed,
                    const Eigen::Vector3d &accFrd,
                    const Eigen::Vector3d &gyroFrd,
                    const Eigen::Vector3d &airspeedFrd,
                    float /*temperature*/,
                    float /*staticPressure*/,
                    float /*diffPressure*/)
    {
        if (!_inited) {
            return -1;
        }
        if (!_ap_online) {
            return 0;
        }
        double rangefinder[6] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
        _libAP.setRangefinder(rangefinder, 6);
        _libAP.setAirspeed(airspeedFrd.norm());
        _libAP.setWindvane(0.0f, 0.0f);

        double lat_deg  = posGeodetic.x();
        double lon_deg  = posGeodetic.y();
        double alt_m    = posGeodetic.z();
        double gx = gyroFrd.x();
        double gy = gyroFrd.y();
        double gz = gyroFrd.z();
        double ax = accFrd.x();
        double ay = accFrd.y();
        double az = accFrd.z();

        Eigen::Vector3d eul = attitudeFrdNed.toRotationMatrix().eulerAngles(0, 1, 2);
        double roll  = eul(0);
        double pitch = eul(1);
        double yaw   = eul(2);

        double vx = velNED.x();
        double vy = velNED.y();
        double vz = velNED.z();

        _libAP.SendState(time_now_sec, gx, gy, gz, ax, ay, az,
                         lat_deg, lon_deg, alt_m,
                         roll, pitch, yaw,
                         vx, vy, vz);
        return 0;
    }

static inline void process(const Eigen::Vector3d &velNED,
                           const Eigen::Vector3d &geodetic_position,
                           const Eigen::Quaterniond &attitudeFrdNed,
                           const Eigen::Vector3d &accFrd,
                           const Eigen::Vector3d &gyroFrd,
                           const Eigen::Vector3d &airspeedFrd,
                           float temperatureCelcius,
                           float staticPressureHpa,
                           float diffPressureHPa,
                           std::vector<double> &actuators,
                           uint8_t &is_armed)
{
    static ArdupilotJsonInterface flight_stack;
    if (!flight_stack._inited) {
        flight_stack.Init("127.0.0.1", 9002);
        return;
    }

    double crnt_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    uint64_t time_usec = static_cast<uint64_t>(crnt_time * 1.0e6);

    bool isArmed;
    std::array<double, 16> servo_cmd{};
    if (flight_stack.Receive(false, isArmed, servo_cmd) == 1) {
        // Use hardcoded actuators setup for the default ArduPlane for a while
        if (actuators.size() != 4) {
            actuators.resize(4);
        }
        actuators[0] = (servo_cmd[0] - 1500.0) / 500.0;     // aileron in [-1..+1]
        actuators[1] = (servo_cmd[1] - 1500.0) / 500.0;     // elevator in [-1..+1]
        actuators[2] = (servo_cmd[2] - 1000.0) / 1000.0;    // throttle in [0..+1]
        actuators[3] = (servo_cmd[3] - 1500.0) / 500.0;     // rudder in [-1..+1]

        is_armed = isArmed ? 1 : 2;
    }

    double time_sec = static_cast<double>(time_usec) / 1e6;
    flight_stack.SendSimData(time_sec,
                             velNED,
                             geodetic_position,
                             attitudeFrdNed,
                             accFrd,
                             gyroFrd,
                             airspeedFrd,
                             temperatureCelcius,
                             staticPressureHpa,
                             diffPressureHPa);
}

private:
    libAP_JSON _libAP;
    bool _inited;
    bool _ap_online;
};

#endif
