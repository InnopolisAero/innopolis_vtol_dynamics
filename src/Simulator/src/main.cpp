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
 */

#include <iostream>
#include "UavDynamics/sim.hpp"
#include "MavlinkSimInterface/mavlink_sim_interface.hpp"
#include "ArdupilotInterface/ardupilot_json_interface.hpp"
#include "RosInterface/ros_interface.hpp"
#include "DronecanInterface/dronecan_interface.hpp"
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>
#include <chrono>
#include <thread>
#include <cstdlib>

void printHelp() {
    std::cout << "Usage: ./UavDynamics-x86_64.AppImage --config <config_file>\n";
    std::cout << "Options:\n";
    std::cout << "  --config    Specify the configuration file (mandatory)\n";
    std::cout << "  --help      Show this help message\n";
}

std::unordered_map<std::string, std::string> parse_args(int argc, char* argv[]) {
    std::unordered_map<std::string, std::string> args;
    std::vector<std::string> arguments(argv + 1, argv + argc);

    for (size_t i = 0; i < arguments.size(); ++i) {
        if (arguments[i] == "--help") {
            printHelp();
            exit(0);
        } else if (arguments[i] == "--config") {
            if (i + 1 < arguments.size()) {
                args["config"] = arguments[i + 1];
                ++i;
            } else {
                throw std::invalid_argument("Missing value for --config");
            }
        } else {
            throw std::invalid_argument("Unknown argument: " + arguments[i]);
        }
    }

    if (args.find("config") == args.end()) {
        throw std::invalid_argument("--config is mandatory");
    }

    return args;
}

double getCurrentTimeInSeconds() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
    return seconds;
}

static std::vector<double> setpoints(16, 0.0);
static ArmingStatus arming_status = ArmingStatus::UNKNOWN;
static std::array<size_t, 6> stats;
static std::shared_ptr<UavDynamics::Sim> sim;

void threadSim() {
    const auto period = std::chrono::microseconds(950); // 1000 Hz
    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        sim->process(setpoints, arming_status);
        stats[4]++;

        auto end_time = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(period - (end_time - start_time));
    }
}

void px4MavlinkCommunicate() {
    float temperatureKelvin;
    float staticPressureHpa;
    float diffPressureHPa;
    sim->atmosphere(&temperatureKelvin, &staticPressureHpa, &diffPressureHPa);

    MavlinkSimInterface::process(
        sim->linearVelocityNed(),
        sim->geoPosition(),
        sim->magneticFieldWithNoiseFrd(),
        sim->getAccelFrd(),
        sim->getGyroFrd(),
        temperatureKelvin - 273.15,
        staticPressureHpa,
        diffPressureHPa,

        setpoints,
        (uint8_t&)arming_status
    );
}

void threadPX4MavlinkFlightStack() {
    const auto period = std::chrono::microseconds(1000000 / 500); // 500 Hz

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        px4MavlinkCommunicate();

        auto end_time = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(period - (end_time - start_time));
    }
}

void threadRos(const std::string& path) {
    RosInterface ros_interface(sim, path);
    ros_interface.run();
}

void threadArdupilotJson() {
    const auto period = std::chrono::microseconds(1000000 / 500); // 500 Hz

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();

        float temperatureKelvin;
        float staticPressureHpa;
        float diffPressureHPa;
        sim->atmosphere(&temperatureKelvin, &staticPressureHpa, &diffPressureHPa);

        ArdupilotJsonInterface::process(
            sim->linearVelocityNed(),
            /*sim->geoPosition()*/ sim->positionNed(), 
            sim->attitudeFrdNed(),
            // sim->magneticFieldWithNoiseFrd(),
            sim->getAccelFrd(),
            sim->getGyroFrd(),
            sim->getAirspeedFrd(),
            temperatureKelvin - 273.15,
            staticPressureHpa,
            diffPressureHPa,

            setpoints,
            (uint8_t&)arming_status
        );
        auto end_time = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(period - (end_time - start_time));
    }
}

int main(int argc, char* argv[]) {
    std::unordered_map<std::string, std::string> args;
    try {
        args = parse_args(argc, argv);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << "\n";
        printHelp();
        return 1;
    }

    auto path = args["config"];
    std::cout << "Dynamics simulator config file: " << path << "\n";

    sim = std::make_shared<UavDynamics::Sim>(path);

    std::cout << "Running the dynamics..." << std::endl;
    std::thread thread1 = std::thread(&threadSim);

    std::cout << "Running the flight stack communicator..." << std::endl;
    std::thread thread2 = std::thread(&threadPX4MavlinkFlightStack);

    std::cout << "Running ROS communicator..." << std::endl;
    std::thread thread3([&path]() {
        threadRos(path);
    });

    std::cout << "Running ArduPilot JSON communicator..." << std::endl;
    std::thread thread4 = std::thread(&threadArdupilotJson);

    std::cout << "Running DroneCAN interface..." << std::endl;
    DronecanInterface::runInSeparateThread(sim, &setpoints);

    thread1.join();
    thread2.join();

    std::cout << "Exit" << std::endl;
    return 0;
}
