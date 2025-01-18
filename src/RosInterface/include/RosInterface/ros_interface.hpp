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

#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/server.hpp>

#include <boost/asio/steady_timer.hpp>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <set>
#include <unordered_map>
#include <stdexcept>
#include <memory>

#include "UavDynamics/sim.hpp"

// ----------------------------------------------------------
// Union for convenience (from your original code).
// ----------------------------------------------------------
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

// ----------------------------------------------------------
// Connection mode enum
// ----------------------------------------------------------
enum class ConnectionMode
{
    CLIENT = 0,
    SERVER
};

// ----------------------------------------------------------
// RosInterface
// ----------------------------------------------------------
class RosInterface
{
public:
    using client_config    = websocketpp::config::asio_client;
    using client_t         = websocketpp::client<client_config>;
    using client_msg_ptr   = client_t::message_ptr;

    using server_config    = websocketpp::config::asio;
    using server_t         = websocketpp::server<server_config>;
    using server_msg_ptr   = server_t::message_ptr;

    /**
     * @brief Constructs the interface by loading YAML config to determine
     *        whether to operate in CLIENT or SERVER mode, plus IP/port, etc.
     *
     * @param sim         Shared pointer to the UAV simulator
     * @param configPath  Path to the YAML file with "connection" info
     */
    RosInterface(std::shared_ptr<UavDynamics::Sim> sim, const std::string& configPath)
        : _sim(sim)
        , _timer(nullptr)
        , _remainingClientReconnectTries(5) // arbitrary
    {
        loadYaml(configPath);
        initializeActuatorMapping();

        // Initialize based on mode
        if (_mode == ConnectionMode::CLIENT) {
            initClient();
        } else {
            initServer();
        }
    }

    /**
     * @brief Starts the I/O loop (blocking).
     *
     * - In CLIENT mode, will call client->run() inside a loop
     *   to re-try if rosbridge disconnects or fails.
     * - In SERVER mode, will attempt to listen, accept, then run() the server.
     */
    void run()
    {
        if (_mode == ConnectionMode::CLIENT) {
            runClientLoop();
        } else {
            runServerLoop();
        }
    }

private:
    // ----------------------- Shared Data -----------------------
    std::shared_ptr<UavDynamics::Sim> _sim;
    float _altitude = 0.0f;
    float _ground_level = 0.0f;

    // Mode & config data
    ConnectionMode _mode;
    std::string _ip;      // e.g. "ws://localhost"
    uint16_t _port = 9090;
    
    // Actuator indices mapped by name
    int _rudder_idx = -1;
    int _ailerons_idx = -1;
    int _elevator_idx = -1;
    int _pusher_motor_idx = -1;

    // ----------------------- CLIENT -----------------------
    std::unique_ptr<client_t> _client;
    std::thread _clientThread;               // For the perpetual I/O loop
    websocketpp::connection_hdl _clientConnection;
    std::unique_ptr<boost::asio::steady_timer> _timer;
    int _remainingClientReconnectTries;

    // ----------------------- SERVER -----------------------
    std::unique_ptr<server_t> _server;
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> _serverConnections;
    bool _serverInitialized = false; // track if we successfully listened

    uint32_t _gpsSeq        = 0;
    uint32_t _actuatorsSeq  = 0;
    uint32_t _attitudeSeq   = 0;

    // =======================================================
    // =============== YAML + Initialization =================
    // =======================================================

    void loadYaml(const std::string& path)
    {
        YAML::Node config = YAML::LoadFile(path);

        if (!config["sim"]) {
            throw std::runtime_error("Missing 'connection' in YAML.");
        }
        auto sim = config["sim"];

        if (!sim["connection-3d"]) {
            throw std::runtime_error("Missing 'connection' in YAML.");
        }
        auto cnode = sim["connection-3d"];
        std::string modeStr = cnode["mode"].as<std::string>();
        if (modeStr == "client") {
            _mode = ConnectionMode::CLIENT;
        } else if (modeStr == "server") {
            _mode = ConnectionMode::SERVER;
        } else {
            throw std::runtime_error("Invalid connection mode in YAML: " + modeStr);
        }

        if (cnode["ip"]) {
            _ip = cnode["ip"].as<std::string>(); 
        } else {
            _ip = "ws://localhost";
        }

        if (cnode["port"]) {
            _port = cnode["port"].as<uint16_t>();
        } else {
            _port = 9090; 
        }

        std::cout << "[RosInterface] Mode: " 
                  << ( (_mode == ConnectionMode::CLIENT) ? "CLIENT" : "SERVER" )
                  << ", IP: " << _ip << ", Port: " << _port << std::endl;
    }

    void initializeActuatorMapping()
    {
        _rudder_idx       = 3;  // ArduPlane SERVO4_FUNCTION
        _ailerons_idx     = 0;  // ArduPlane SERVO1_FUNCTION
        _elevator_idx     = 1;  // ArduPlane SERVO2_FUNCTION
        _pusher_motor_idx = 2;  // ArduPlane SERVO3_FUNCTION

        std::cout << "[RosInterface] Found actuators mapping -> "
                  << "rudder=" << _rudder_idx << ", "
                  << "aileron=" << _ailerons_idx << ", "
                  << "elevator=" << _elevator_idx << ", "
                  << "pusher=" << _pusher_motor_idx << std::endl;
    }

    // =======================================================
    // ====================== CLIENT =========================
    // =======================================================

    void initClient() {
        std::cout << "[RosInterface] Initializing CLIENT mode -> " 
                << _ip << ":" << _port << std::endl;

        _client = std::make_unique<client_t>();

        // 1) "Perpetual" mode: The ASIO loop does not stop after a fail or close
        _client->init_asio();
        _client->start_perpetual(); // Important

        // Disable (or enable) logging as needed
        _client->clear_access_channels(websocketpp::log::alevel::all);
        _client->clear_error_channels(websocketpp::log::elevel::all);

        // Set handlers
        //  - on_open: Once handshake succeeds
        //  - on_fail: Called if handshake fails
        //  - on_close: Called if the connection closes
        _client->set_open_handler([this](websocketpp::connection_hdl hdl) {
            onClientOpen(hdl);
        });
        _client->set_message_handler([this](websocketpp::connection_hdl hdl, client_msg_ptr msg) {
            onClientMessage(hdl, msg);
        });
        _client->set_fail_handler([this](websocketpp::connection_hdl hdl) {
            onClientFail(hdl);
        });
        _client->set_close_handler([this](websocketpp::connection_hdl hdl) {
            onClientClose(hdl);
        });

        // 2) Start a dedicated thread to run() the I/O loop
        _clientThread = std::thread([this]() {
            try {
                _client->run(); 
            } catch (const std::exception& e) {
                std::cerr << "[ROS-CLIENT] I/O thread exception: " << e.what() << std::endl;
            }
            std::cout << "[ROS-CLIENT] I/O thread exited.\n";
        });
    }

    /**
     * @brief Actually attempts to connect to the rosbridge server.
     *        Called from runClientLoop() or re-called on fail.
     */
    void tryClientConnect()
    {
        std::string uri = _ip + ":" + std::to_string(_port);
        std::cout << "[ROS-CLIENT] Attempting async connect to " << uri << std::endl;

        websocketpp::lib::error_code ec;
        client_t::connection_ptr con = _client->get_connection(uri, ec);
        if (ec) {
            std::cerr << "[ROS-CLIENT] get_connection failed: " << ec.message() << std::endl;
            return;
        }

        _clientConnection = con->get_handle();
        _client->connect(con);
        // No blocking; handshake runs asynchronously in the I/O thread
    }

    /**
     * @brief Loops calling _client->run() which blocks until disconnected or error.
     *        Retries a limited number of times if fails.
     */
    void runClientLoop()
    {
        const int retryDelaySeconds = 2;
        const int maxRetries = -1; // infinite
        int retryCount = 0;

        while (maxRetries < 0 || retryCount < maxRetries) {
            // Call it, ignoring any return value
            tryClientConnect();

            std::cout << "[ROS-CLIENT] Starting ASIO loop...\n";
            try {
                _client->run(); // blocks until fail/close
            } catch (const websocketpp::exception& e) {
                std::cerr << "[ROS-CLIENT] run() exception: " << e.what() << std::endl;
            }

            std::cerr << "[ROS-CLIENT] Connection ended or failed. Retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(retryDelaySeconds));
            retryCount++;
        }

        std::cerr << "[ROS-CLIENT] Exceeded maximum retries. Exiting client loop.\n";
    }
    
    // ------------------ Client Handlers ------------------
    void onClientOpen(websocketpp::connection_hdl hdl)
    {
        std::cout << "[ROS-CLIENT] Connected to rosbridge.\n";
        _clientConnection = hdl;

        // Subscribe to altimeter info
        subscribeTopic("/sensor/altimeter", "sensor_msgs/Range");
        subscribeTopic("/sensor/altimeter/ground_level", "std_msgs/Float32");

        // Optionally advertise our out-going topics
        advertiseTopic("/sim/attitude",      "geometry_msgs/QuaternionStamped");
        advertiseTopic("/sim/actuators",     "sensor_msgs/Joy");
        advertiseTopic("/sim/gps_position",  "sensor_msgs/NavSatFix");

        // Start a 25Hz timer to publish periodically
        startPublishTimer();
    }

    void onClientMessage(websocketpp::connection_hdl, client_msg_ptr msg)
    {
        // Parse JSON
        try {
            auto j = nlohmann::json::parse(msg->get_payload());
            if (j.contains("op") && j["op"] == "publish") {
                // altimeter
                if (j.contains("topic") && j["topic"] == "/sensor/altimeter") {
                    if (j.contains("msg") && j["msg"].contains("range")) {
                        _altitude = j["msg"]["range"].get<double>();
                    }
                }
                // ground level
                else if (j.contains("topic") && j["topic"] == "/sensor/altimeter/ground_level") {
                    if (j.contains("msg") && j["msg"].contains("data")) {
                        _ground_level = j["msg"]["data"].get<double>();
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[ROS-CLIENT] onClientMessage parse error: " << e.what() << std::endl;
        }
    }

    void onClientFail(websocketpp::connection_hdl hdl)
    {
        std::cerr << "[ROS-CLIENT] Connection failed. Probably rosbridge not up yet.\n";

        // Wait a bit before trying again (avoid spamming)
        std::this_thread::sleep_for(std::chrono::seconds(2));
        tryClientConnect();
    }

    void onClientClose(websocketpp::connection_hdl hdl)
    {
        std::cerr << "[ROS-CLIENT] Connection closed.\n";

        // Possibly reconnect as well
        std::this_thread::sleep_for(std::chrono::seconds(2));
        tryClientConnect();
    }

    // =======================================================
    // ====================== SERVER =========================
    // =======================================================

    void initServer()
    {
        std::cout << "[RosInterface] Initializing SERVER mode on port " << _port << std::endl;

        _server = std::make_unique<server_t>();
        _server->init_asio();

        // Let the server reuse the port if it was in TIME_WAIT, etc.
        _server->set_reuse_addr(true);

        // Optionally disable logging
        _server->clear_access_channels(websocketpp::log::alevel::all);
        _server->clear_error_channels(websocketpp::log::elevel::all);

        // Set handlers
        _server->set_open_handler([this](websocketpp::connection_hdl hdl) {
            onServerOpen(hdl);
        });
        _server->set_close_handler([this](websocketpp::connection_hdl hdl) {
            onServerClose(hdl);
        });
        _server->set_message_handler([this](websocketpp::connection_hdl hdl, server_msg_ptr msg) {
            onServerMessage(hdl, msg);
        });
    }

    /**
     * @brief Listen/bind to our port with a few retries,
     *        then start_accept(), then run().
     */
    void runServerLoop()
    {
        const int maxAttempts = 5;
        const int retryDelaySeconds = 2;
        bool success = false;

        std::string bindIp = "0.0.0.0"; 
        // or "127.0.0.1" if you only want local connections
        // If your `_ip` is "ws://localhost", we typically ignore that 
        // for the underlying bind operation, and just do 0.0.0.0.

        for (int attempt = 1; attempt <= maxAttempts; ++attempt) {
            try {
                // Explicitly create an endpoint
                boost::asio::ip::tcp::endpoint endpoint(
                    boost::asio::ip::make_address(bindIp), 
                    _port
                );

                // Listen on that endpoint
                _server->listen(endpoint); 
                // Then accept
                _server->start_accept();

                success = true;
                std::cout << "[SERVER-CLIENT] Successfully bound to " 
                        << bindIp << ":" << _port << std::endl;
                break; 
            } catch (websocketpp::exception const &e) {
                std::cerr << "[SERVER-CLIENT] Listen attempt " 
                        << attempt << " failed: " << e.what() << std::endl;

                if (attempt < maxAttempts) {
                    std::cerr << "[SERVER-CLIENT] Will retry in " 
                            << retryDelaySeconds << "s...\n";

                    // Recreate the server object in case it got into a bad state
                    resetServer();
                    std::this_thread::sleep_for(std::chrono::seconds(retryDelaySeconds));

                    // IMPORTANT: re-apply reuse_addr & set handlers in resetServer()
                }
            }
        }

        if (!success) {
            std::cerr << "[SERVER-CLIENT] Could not bind port after " 
                    << maxAttempts << " attempts. Exiting.\n";
            return;
        }

        std::cout << "[SERVER-CLIENT] Starting ASIO loop...\n";
        startPublishTimer();
        try {
            _server->run();
        } catch (const websocketpp::exception &e) {
            std::cerr << "[SERVER-CLIENT] run() exception: " << e.what() << std::endl;
        }
    }

    /**
     * @brief Recreate or reset the server so we can retry listen().
     */
    void resetServer()
    {
        _server.reset(new server_t());
        _server->init_asio();

        // Reuse address again
        _server->set_reuse_addr(true);

        // Disable logging again if desired
        _server->clear_access_channels(websocketpp::log::alevel::all);
        _server->clear_error_channels(websocketpp::log::elevel::all);

        // Re-bind our handlers
        _server->set_open_handler([this](auto hdl) { onServerOpen(hdl); });
        _server->set_close_handler([this](auto hdl) { onServerClose(hdl); });
        _server->set_message_handler([this](auto hdl, server_msg_ptr msg) {
            onServerMessage(hdl, msg);
        });
    }
    
    // ------------------ Server Handlers ------------------
    void onServerOpen(websocketpp::connection_hdl hdl)
    {
        _serverConnections.insert(hdl);
        std::cout << "[SERVER-CLIENT] Client connected. Total: " << _serverConnections.size() << std::endl;
    }

    void onServerClose(websocketpp::connection_hdl hdl)
    {
        _serverConnections.erase(hdl);
        std::cout << "[SERVER-CLIENT] Client disconnected. Total: " << _serverConnections.size() << std::endl;
    }

    void onServerMessage(websocketpp::connection_hdl hdl, server_msg_ptr msg)
    {
        try {
            auto j = nlohmann::json::parse(msg->get_payload());
            if (j.contains("op") && j["op"] == "publish") {
                // altimeter
                if (j.contains("topic") && j["topic"] == "/sensor/altimeter") {
                    if (j.contains("msg") && j["msg"].contains("range")) {
                        _altitude = j["msg"]["range"].get<double>();
                    }
                }
                // ground level
                else if (j.contains("topic") && j["topic"] == "/sensor/altimeter/ground_level") {
                    if (j.contains("msg") && j["msg"].contains("data")) {
                        _ground_level = j["msg"]["data"].get<double>();
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[SERVER-CLIENT] onServerMessage parse error: " << e.what() << std::endl;
        }
    }

    // =======================================================
    // =================== PERIODIC PUBLISH ==================
    // =======================================================
    void startPublishTimer()
    {
        using namespace std::chrono_literals;

        // Use the correct io_service from client or server
        auto& ios = (_mode == ConnectionMode::CLIENT) 
                    ? _client->get_io_service()
                    : _server->get_io_service();

        _timer = std::make_unique<boost::asio::steady_timer>(ios, 40ms);
        _timer->async_wait([this](const boost::system::error_code& ec) {
            if (!ec) {
                publishAllTopics();
                startPublishTimer(); // schedule again
            }
        });
    }

    void publishAllTopics()
    {
        publishAttitude();
        publishActuators();
        publishGPS();
    }

    // Creates a JSON "header" with seq, stamp, frame_id in the standard ROS format
    inline nlohmann::json makeRosHeader(uint32_t seq, const std::string& frame_id)
    {
        // If you have a sim time, or real clock:
        auto now = std::chrono::system_clock::now().time_since_epoch();
        uint64_t nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

        uint32_t secs = static_cast<uint32_t>(nanos / 1000000000ULL);
        uint32_t nsecs = static_cast<uint32_t>(nanos % 1000000000ULL);

        return {
            {"seq", seq},
            {"stamp", {
                {"secs",  secs},
                {"nsecs", nsecs}
            }},
            {"frame_id", frame_id}
        };
    }

    // -------------------------------------------------------
    //  "publish" methods
    // -------------------------------------------------------
    void publishAttitude()
    {
        auto quat = _sim->attitudeFluEnu();
        
        nlohmann::json header = makeRosHeader(_attitudeSeq++, "base_link");

        nlohmann::json msg {
            {"header", header},
            {"quaternion", {
                {"x", quat.x()},
                {"y", quat.y()},
                {"z", quat.z()},
                {"w", quat.w()}
            }}
        };
        publishMessage("/sim/attitude", msg);
    }

    void publishActuators()
    {
        // Your existing code that creates the "axes", "buttons", etc.
        OctoplaneVtolAirframeIn3dSim airframe = {
            .controls = {
                .motor_front_right = 0.0,
                .motor_rear_left   = 0.0,
                .motor_front_left  = 0.0,
                .motor_rear_right  = 0.0,
                .pusher_motor      = _sim->getActuatorStateScaled(_pusher_motor_idx),
                .ailerons          = -(_sim->getActuatorStateScaled(_ailerons_idx)),
                .elevator          =  (_sim->getActuatorStateScaled(_elevator_idx)),
                .rudder            =  (_sim->getActuatorStateScaled(_rudder_idx))
            }
        };

        nlohmann::json header = makeRosHeader(_actuatorsSeq++, "base_link");

        nlohmann::json msg = {
            {"header", header},
            {"axes",    std::vector<double>(std::begin(airframe.values), std::end(airframe.values))},
            {"buttons", {1, 0, 0, 1}}
        };

        publishMessage("/sim/actuators", msg);
    }

    void publishGPS()
    {
        // Compute or fetch your current GPS coords, altitude, etc. as before
        auto gpsPosition = _sim->geoPosition();
        auto posNed = _sim->positionNed(); 
        posNed[2] = 0.0;
        auto gpsGround = _sim->geoPositionFromNed(posNed);
        double correctedAlt = gpsPosition[2] + _ground_level - gpsGround[2];

        nlohmann::json header = makeRosHeader(_gpsSeq++, "gps");

        nlohmann::json msg = {
            {"header", header},
            // "status" sub-message from sensor_msgs/NavSatStatus
            {"status", {
                {"status", 0},   // STATUS_FIX = 0
                {"service", 0}   // SERVICE_GPS = 1, but 0 is valid if you have none
            }},
            {"latitude",  gpsPosition[0]},
            {"longitude", gpsPosition[1]},
            {"altitude",  correctedAlt},
            {"position_covariance", {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1}},
            {"position_covariance_type", 2}
        };
        publishMessage("/sim/gps_position", msg);
    }

    // -------------------------------------------------------
    // Publish logic (either client->send or server->broadcast)
    // -------------------------------------------------------
    void publishMessage(const std::string& topic, const nlohmann::json& content)
    {
        nlohmann::json out = {
            {"op",    "publish"},
            {"topic", topic},
            {"msg",   content}
        };

        if (_mode == ConnectionMode::CLIENT) {
            // send to rosbridge
            if (_client) {
                try {
                    _client->send(_clientConnection, out.dump(), websocketpp::frame::opcode::text);
                } catch (const websocketpp::exception& e) {
                    std::cerr << "[ROS-CLIENT] publishMessage error: " << e.what() << std::endl;
                }
            }
        } else {
            // broadcast to all connected sim clients
            if (_server) {
                auto payload = out.dump();
                for (auto& conn : _serverConnections) {
                    try {
                        _server->send(conn, payload, websocketpp::frame::opcode::text);
                        // std::cout << "[ROS-SERVER] publishMessage: " << " " << payload << std::endl;
                    } catch (const websocketpp::exception& e) {
                        std::cerr << "[SERVER-CLIENT] broadcast error: " << e.what() << std::endl;
                    }
                }
            }
        }
    }

    // -------------------------------------------------------
    // (Optional) These only apply to client mode
    // -------------------------------------------------------
    void subscribeTopic(const std::string& topic, const std::string& type)
    {
        nlohmann::json msg {
            {"op",    "subscribe"},
            {"topic", topic},
            {"type",  type}
        };
        sendClientMsg(msg);
    }

    void advertiseTopic(const std::string& topic, const std::string& type)
    {
        nlohmann::json msg {
            {"op",    "advertise"},
            {"topic", topic},
            {"type",  type}
        };
        sendClientMsg(msg);
    }

    void sendClientMsg(const nlohmann::json& msg)
    {
        if (!_client) return;
        try {
            _client->send(_clientConnection, msg.dump(), websocketpp::frame::opcode::text);
        } catch (const websocketpp::exception& e) {
            std::cerr << "[ROS-CLIENT] sendClientMsg error: " << e.what() << std::endl;
        }
    }
};

#endif // ROS_INTERFACE_HPP
