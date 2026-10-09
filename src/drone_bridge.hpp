#ifndef DRONE_BRIDGE_HPP
#define DRONE_BRIDGE_HPP

#include <mosquittopp.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <string>
#include <cstdint>
#include <chrono>
#include <atomic>

using json = nlohmann::json;

// config loaded from mosquitto.conf at startup
// these are only default values
struct BridgeConfig {
    std::string broker_ip = "localhost";
    int broker_port = 8883;
    std::string username;
    std::string password;
    std::string ca_cert_path;
    std::string manufacturer = "DefaultCorp";
    std::string serial_number = "drone01";
    std::string version = "1.0.0";
    int publish_interval = 1000; // in miliseconds
    int heartbeat_timeout = 3; // in seconds
    std::string bind_address = "127.0.0.1";
    int udp_port = 14551;
};

BridgeConfig load_config(const std::string& path);

class DroneBridge : public mosqpp::mosquittopp {
public:
    DroneBridge(const BridgeConfig& cfg);
    ~DroneBridge();

    // start the bridge: launches MAVLink thread, runs MQTT publish loop (blocking)
    void run();

    // shutdown: publishes offline, stops mosquitto loop
    void shutdown();

private:
    // MQTT callbacks
    void on_connect(int return_code) override;
    void on_disconnect(int return_code) override;

    void mavlink_receive_loop();

    static const char* copter_mode_name(uint32_t mode);

    void publish_loop();

    // JSON builders
    std::string build_telemetry_payload();
    std::string build_connection_payload(const std::string& state = "ONLINE");
    std::string build_lwt_payload();

    // utils functions
    static std::string get_iso8601_timestamp();
    std::string topic_prefix() const;

    // MQTT config
    BridgeConfig cfg_;

    // telemetry (written by MAVLink thread, read by MQTT thread)
    std::mutex data_mutex_;

    // position
    // latitude and longitude scaled by 1e7 in docs
    // conversion back to degrees needs to scaled back
    double lat_ = 0.0; 
    double lon_ = 0.0; 
    double alt_ = 0.0; // mm -> meters

    // velocity in NED frame
    double vn_ = 0.0; // m/s north
    double ve_ = 0.0; // m/s east
    double vd_ = 0.0; // m/s down (positive = descending)

    int battery_pct_ = -1; // 0..100 %, -1 = unknown
    uint16_t voltage_mV_  = 0; // millivolts, UINT16_MAX = unknown
    int16_t current_cA_  = 0; // centi-amps, -1 = unknown

    // attitude (rad) and angular velocity (rad/s)
    float roll_ = 0; // -pi to pi;
    float pitch_ = 0;
    float yaw_ = 0;
    float rollspeed_ = 0;
    float pitchspeed_ = 0;
    float yawspeed_ = 0;

    bool armed_ = false;
    uint32_t flight_mode_num_ = 0;
    std::string flight_mode_ = "UNKNOWN";

    // heartbeat
    std::chrono::steady_clock::time_point last_heartbeat_;
    bool drone_online_ = false;

    // header
    uint32_t telemetry_msg_id_  = 0;
    uint32_t connection_msg_id_ = 0;

    std::atomic<bool> running_{true};
};

#endif // DRONE_BRIDGE_HPP