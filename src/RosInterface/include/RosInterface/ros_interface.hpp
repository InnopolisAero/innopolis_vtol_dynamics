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
#ifndef ROS_INTERFACE_ROS_INTERFACE_HPP
#define ROS_INTERFACE_ROS_INTERFACE_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>
#include <memory>
#include "UavDynamics/sim.hpp"

union OctoplaneVtolAirframeIn3dSim {
    struct {
        double motor_front_right;
        double motor_rear_left;
        double motor_front_left;
        double motor_rear_right;

        double pusher_motor;

        double ailerons;
        double elevator;
        double rudder;

        void print()
        {
            std::cout << "Actuators: ["
                      << motor_front_right << ", "
                      << motor_rear_left << ", "
                      << motor_front_left << ", "
                      << motor_rear_right << ", "
                      << pusher_motor << "], ("
                      << ailerons << ", "
                      << elevator << ", "
                      << rudder << ")" << std::endl;
        }
    } controls;

    double values[sizeof(controls) / sizeof(double)];
};

class RosPublisher {
public:
    RosPublisher(std::shared_ptr<UavDynamics::Sim> sim, const std::string& uri = "ws://localhost:9090") :
        _client(),
        _server_uri(uri),
        _timer(nullptr),
        _sim(sim)
    {
        _client.init_asio();

        // Disable all logging
        _client.clear_access_channels(websocketpp::log::alevel::all);
        _client.clear_error_channels(websocketpp::log::elevel::all);

        // Set WebSocket handlers
        _client.set_open_handler([this](websocketpp::connection_hdl hdl) {
            this->on_open(hdl);
        });
        _client.set_message_handler([this](websocketpp::connection_hdl hdl, message_ptr msg) {
            try {
                nlohmann::json data = nlohmann::json::parse(msg->get_payload());

                // Debugging: Log the entire message payload
                // std::cout << "[DEBUG] Received message: " << data.dump(4) << std::endl;

                // Check if it's a published message
                if (data.contains("op") && data["op"] == "publish") {
                    // Check the topic name
                    if (data.contains("topic") && data["topic"] == "/sensor/altimeter") {
                        // Parse the "range" field from sensor_msgs/Range
                        if (data.contains("msg") && data["msg"].contains("range")) {
                            _altitude = data["msg"]["range"].get<double>();

                            // Update sim altitude
                            // _sim->setRangefinderAltitude(_altitude);

                            // std::cout << "[RosPublisher] Received altitude: " << _altitude << std::endl;
                        } else {
                            std::cerr << "[ERROR] /sensor/altimeter message does not contain 'range' field" << std::endl;
                        }
                    }
                    else if (data.contains("topic") && data["topic"] == "/sensor/altimeter/ground_level")
                    {
                        // This is the "std_msgs/Float32" data
                        if (data.contains("msg") && data["msg"].contains("data"))
                        {
                            _ground_level = data["msg"]["data"].get<double>(); 
                            // or float groundLevel = data["msg"]["data"].get<float>();

                            // _sim->setGroundLevelFromGeodeticAlt(_ground_level);

                            // std::cout << "[DEBUG] Ground level: " << _ground_level << std::endl;
                        }
                        else {
                            std::cerr << "[ERROR] /sensor/altimeter/ground_level message missing 'data' field!" << std::endl;
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception while parsing message: " << e.what() << std::endl;
            }
        });
        _client.set_close_handler([this](websocketpp::connection_hdl hdl) {
            std::cout << "Connection closed." << std::endl;
        });
        _client.set_fail_handler([this](websocketpp::connection_hdl hdl) {
            std::cerr << "Connection failed." << std::endl;
        });

        websocketpp::lib::error_code ec;
        auto con = _client.get_connection(_server_uri, ec);
        if (ec) {
            std::cerr << "Connection failed: " << ec.message() << std::endl;
            return;
        }
        _client.connect(con);
    }

    void run() {
        _client.run();
    }

private:
    float _altitude;
    float _ground_level;

    using client_t = websocketpp::client<websocketpp::config::asio_client>;
    using message_ptr = client_t::message_ptr;

    client_t _client;
    std::string _server_uri;
    websocketpp::connection_hdl connection_;
    std::unique_ptr<boost::asio::steady_timer> _timer;
    std::shared_ptr<UavDynamics::Sim> _sim;

    void on_open(websocketpp::connection_hdl hdl) {
        connection_ = hdl;
        std::cout << "Connected to rosbridge server at " << _server_uri << std::endl;

        // Advertise the topics
        advertise_topic("/sim/attitude", "geometry_msgs/QuaternionStamped");
        advertise_topic("/sim/actuators", "sensor_msgs/Joy");
        advertise_topic("/sim/gps_position", "sensor_msgs/NavSatFix");

        // Subscribe
        subscribe_topic("/sensor/altimeter", "sensor_msgs/Range");
        subscribe_topic("/sensor/altimeter/ground_level", "std_msgs/Float32");

        // Start periodic publishing
        start_timer();
    }

    void start_timer() {
        _timer = std::make_unique<boost::asio::steady_timer>(
            _client.get_io_service(), 
            std::chrono::milliseconds(40) // 25 Hz
        );
        _timer->async_wait([this](const boost::system::error_code& ec) {
            if (!ec) {
                this->publish_messages();
                this->start_timer();
            }
        });
    }

    void advertise_topic(const std::string& topic, const std::string& type) {
        nlohmann::json advertise_msg = {
            {"op", "advertise"},
            {"topic", topic},
            {"type", type}
        };
        try {
            _client.send(connection_, advertise_msg.dump(), websocketpp::frame::opcode::text);
            std::cout << "Advertised topic: " << topic << " with type: " << type << std::endl;
        } catch (const websocketpp::exception& e) {
            std::cerr << "Error advertising topic: " << e.what() << std::endl;
        }
    }

    void subscribe_topic(const std::string& topic, const std::string& type) {
        nlohmann::json subscribe_msg = {
            {"op", "subscribe"},
            {"topic", topic},
            {"type", type}
        };
        try {
            _client.send(connection_, subscribe_msg.dump(), websocketpp::frame::opcode::text);
            std::cout << "Subscribed to topic: " << topic 
                      << " with type: " << type << std::endl;
        } catch (const websocketpp::exception& e) {
            std::cerr << "Error subscribing to topic: " << e.what() << std::endl;
        }
    }

    void publish_messages() {
        publish_attitude();
        publish_actuators();
        publish_gps_position();
    }

    void publish_attitude() {
        auto attitude = _sim->attitudeFluEnu();
        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "base_link"}}},
            {"quaternion", {
                {"x", attitude.x()}, 
                {"y", attitude.y()}, 
                {"z", attitude.z()}, 
                {"w", attitude.w()}
            }}
        };
        publish_message("/sim/attitude", msg);
    }

    void publish_actuators() {
        // OctoplaneVtolAirframeIn3dSim airframe = {
        //     .controls = {
        //         .motor_front_right = _sim->getActuatorState(0) * (60 / 2 / 3.14),
        //         .motor_rear_left = _sim->getActuatorState(1) * (60 / 2 / 3.14),
        //         .motor_front_left = _sim->getActuatorState(2) * (60 / 2 / 3.14),
        //         .motor_rear_right = _sim->getActuatorState(3) * (60 / 2 / 3.14),

        //         .ailerons = 1.5 * _sim->getActuatorState(5) * (180 / 3.14),
        //         .right_aileron = 1.5 * _sim->getActuatorState(6) * (180 / 3.14),
        //         .elevator = 1.5 * _sim->getActuatorState(7) * (180 / 3.14),
        //         .rudder = +50,

        //         .pusher_motor = _sim->getActuatorState(4) * (60 / 2 / 3.14),
        //     }
        // };

        // In actuators we have motors velocity in rad/s and servos position in rad for aerodynamic control surfaces
        // 3d sim gets values in range -1 to 1 and scales intrinsic

        OctoplaneVtolAirframeIn3dSim airframe = {
            .controls = {
                .motor_front_right = 0.0,
                .motor_rear_left = 0.0,
                .motor_front_left = 0.0,
                .motor_rear_right = 0.0,

                .pusher_motor = _sim->getActuatorStateScaled(2),

                .ailerons = _sim->getActuatorStateScaled(0),
                .elevator = _sim->getActuatorStateScaled(1),
                .rudder = _sim->getActuatorStateScaled(3)
            }
        };

        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "base_link"}}},
            {"buttons", {1, 0, 0, 1}}
        };
        msg["axes"] = std::vector<double>(std::begin(airframe.values), std::end(airframe.values));

        // airframe.controls.print();
        publish_message("/sim/actuators", msg);
    }

    void publish_gps_position() {
        auto gps = _sim->geoPosition();
        auto position_ned_ground = _sim->positionNed();
        position_ned_ground[2] = 0;
        auto gps_ground = _sim->geoPositionFromNed(position_ned_ground);
        // std::cout << "gps[2] " << gps[2] << " gps_ground[2] " << gps_ground[2] << " _ground_level " << _ground_level << std::endl;
        nlohmann::json msg = {
            {"header", {{"stamp", {{"sec", 12345}, {"nsec", 67890}}}, {"frame_id", "gps"}}},
            {"latitude", gps[0]},
            {"longitude", gps[1]},
            {"altitude", gps[2] + _ground_level - gps_ground[2]},
            {"position_covariance", {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1}},
            {"position_covariance_type", 2}
        };
        publish_message("/sim/gps_position", msg);
    }

    void publish_message(const std::string& topic, const nlohmann::json& message) {
        nlohmann::json publish_msg = {
            {"op", "publish"},
            {"topic", topic},
            {"msg", message}
        };
        try {
            _client.send(connection_, publish_msg.dump(), websocketpp::frame::opcode::text);
        } catch (const websocketpp::exception& e) {
            std::cerr << "Error sending message: " << e.what() << std::endl;
        }
    }
};

#endif  // ROS_INTERFACE_ROS_INTERFACE_HPP
