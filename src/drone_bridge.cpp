#include "drone_bridge.hpp"
 
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <csignal>
#include <atomic>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
 
#include <ardupilotmega/mavlink.h>

static std::atomic<bool> g_stop{false};

extern "C" void on_signal(int) {
    g_stop = true;
}


BridgeConfig load_config(const std::string& path) {
    BridgeConfig cfg;
    std::ifstream config_file (path);

    if (!config_file.is_open()) {
        printf("Cannot open config file\n");
        return cfg;
    }

    try {
        json mqtt_config = json::parse(config_file);

        for (const char* k : {"broker_ip", "broker_port", "username", "password",
                            "ca_cert_path", "manufacturer", "serial_number", "version",
                            "publish_interval", "heartbeat_timeout", "bind_address", "udp_port"}) {
            if (!mqtt_config.contains(k))
                fprintf(stderr, "config: '%s' missing, using default\n", k);
        }

        cfg.broker_ip = mqtt_config.value("broker_ip", cfg.broker_ip);
        cfg.broker_port = mqtt_config.value("broker_port", cfg.broker_port);
        cfg.username = mqtt_config.at("username");
        cfg.password = mqtt_config.at("password");
        cfg.ca_cert_path = mqtt_config.at("ca_cert_path");
        cfg.manufacturer = mqtt_config.value("manufacturer", cfg.manufacturer);
        cfg.serial_number = mqtt_config.value("serial_number", cfg.serial_number);
        cfg.version = mqtt_config.value("version", cfg.version);
        cfg.publish_interval = mqtt_config.value("publish_interval", cfg.publish_interval);
        cfg.heartbeat_timeout = mqtt_config.value("heartbeat_timeout", cfg.heartbeat_timeout);
        cfg.bind_address = mqtt_config.value("bind_address", cfg.bind_address);
        cfg.udp_port = mqtt_config.value("udp_port", cfg.udp_port);

    } catch (const json::exception &e){
        fprintf(stderr, "Config error: %s (using defaults for the rest)\n", e.what());
    }

    return cfg;
}

DroneBridge::DroneBridge(const BridgeConfig& cfg)
    : mosqpp::mosquittopp((cfg.serial_number + "_drone_bridge").c_str()),
      cfg_(cfg)
{
    mosqpp::lib_init();


    if (!cfg_.username.empty()) {
        username_pw_set(cfg_.username.c_str(), cfg_.password.c_str());
    }


    if (!cfg.ca_cert_path.empty()) {
        // tls_set(cafile, capath, certfile, keyfile, pw_callback)
        int tls_return_code = this->tls_set(cfg.ca_cert_path.c_str(), nullptr, nullptr, nullptr, nullptr);
        if (tls_return_code == MOSQ_ERR_SUCCESS) {
            printf("TLS configured via CA: %s\n", cfg.ca_cert_path.c_str());
        } else {
            printf("TLS setup failed, error: %d\n", tls_return_code);
        }
    }

    this->reconnect_delay_set(2, 30, true);

    std::string conn_topic = topic_prefix() + "/connection";
    std::string lwt = build_lwt_payload();
    this->will_set(conn_topic.c_str(), lwt.length(), lwt.c_str(), 1, true);


    // async connection to MQTT broker
    int conn_res = this->connect_async(cfg.broker_ip.c_str(), cfg.broker_port); // default keep_alive(3rd argument) is 60

    if (conn_res == MOSQ_ERR_SUCCESS) {
        printf("MQTT async connection initiated to IP:PORT: {%s:%d}\n", cfg.broker_ip.c_str(), cfg.broker_port);
    } else {
        printf("MQTT async connection failed, error: %d\n", conn_res);
    }

    this->loop_start(); // starts MQTT network thread
}

DroneBridge::~DroneBridge() {
    shutdown();
}

void DroneBridge::run() {

    std::thread mavlink_thread(&DroneBridge::mavlink_receive_loop, this);
    mavlink_thread.detach();

    printf("MAVLink bridge running on UDP %d, MQTT to %s:%d\n",
           cfg_.udp_port, cfg_.broker_ip.c_str(), cfg_.broker_port);


    this->publish_loop();
}

void DroneBridge::shutdown() {

    if (!this->running_) return; // already shut down
    this->running_ = false;

    std::string conn_topic = topic_prefix() + "/connection";
    std::string payload = build_connection_payload("OFFLINE");
    this->publish(nullptr, conn_topic.c_str(), payload.length(), payload.c_str(), 1, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    this->loop_stop(true);
    this->disconnect();
    mosqpp::lib_cleanup();

    printf("Published OFFLINE and shut down gracefully.\n");

}

void DroneBridge::on_connect(int return_code) {
    if (return_code == 0) {
        printf("MQTT connection success\n");

        std::string conn_topic = topic_prefix() + "/connection";
        std::string payload = build_connection_payload("ONLINE");
        // initial connection message so MQTT needs to retain info and ensure connection is established
        this->publish(nullptr, conn_topic.c_str(), payload.length(), payload.c_str(), 1, true);
    
    } else {
        printf("MQTT connection failed, return code:%d\n", return_code);
    }
}

void DroneBridge::on_disconnect(int return_code) {
    if (return_code != 0) {
        printf("MQTT connection lost, return code: %d. Auto-reconnecting...\n", return_code);
    } else {
        printf("MQTT disconnected\n");
    }
}

void DroneBridge::publish_loop() {
    while(running_ && !g_stop) {
        // locking thread due to mavlink receiver accessing heartbeat
        // at the same time as mqtt publisher
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat_).count();

            if (drone_online_ && elapsed >= cfg_.heartbeat_timeout) {
                drone_online_ = false;
                printf("Timeout (%llds), drone is now offline\n", (long long)elapsed);
            } 
        }

        std::string topic   = topic_prefix() + "/telemetry";
        std::string payload = build_telemetry_payload();
        // telemetry so MQTT doesn't need to retain connection messages (false)
        // and 0 Quality of service due to rapid information
        this->publish(nullptr, topic.c_str(), payload.length(), payload.c_str(), 0, false);
 
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.publish_interval));

    }
}

std::string DroneBridge::build_telemetry_payload() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    json telemetry_payload;
    telemetry_payload["header_id"] = telemetry_msg_id_++;
    telemetry_payload["timestamp"] = get_iso8601_timestamp();
    telemetry_payload["version"] = cfg_.version;
    telemetry_payload["manufacturer"] = cfg_.manufacturer;
    telemetry_payload["serial_number"] = cfg_.serial_number;

    telemetry_payload["position"] = {
        {"lat", lat_},
        {"lon", lon_},
        {"alt", alt_}
    };

    telemetry_payload["velocity"] = {
        {"vn", vn_},
        {"ve", ve_},
        {"vd", vd_}
    };

        // orientation in radians (roll/pitch/yaw, -pi..pi)
    telemetry_payload["attitude"] = {
        {"roll", roll_},
        {"pitch", pitch_},
        {"yaw", yaw_}
    };
 
    // angular rates in rad/s
    telemetry_payload["angular_velocity"] = {
        {"rollspeed", rollspeed_},
        {"pitchspeed", pitchspeed_},
        {"yawspeed", yawspeed_}
    };

    telemetry_payload["battery_state"] = {
        {"batteryCharge", (battery_pct_ < 0) ? json(nullptr) : json(battery_pct_)},
        {"voltage", voltage_mV_ / 1000.0},
        {"current", current_cA_ / 100.0}
    };

    telemetry_payload["armed"] = armed_;
    telemetry_payload["flight_mode"] = flight_mode_;
    telemetry_payload["flight_mode_num"] = flight_mode_num_;
 
    // status
    telemetry_payload["online"] = drone_online_;

    return telemetry_payload.dump();
}

std::string DroneBridge::build_connection_payload(const std::string& state) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    json conn_payload;
    conn_payload["header_id"]        = connection_msg_id_++;
    conn_payload["timestamp"]       = get_iso8601_timestamp();
    conn_payload["version"]         = cfg_.version;
    conn_payload["manufacturer"]    = cfg_.manufacturer;
    conn_payload["serial_number"]    = cfg_.serial_number;
    conn_payload["connection_state"] = state;
    return conn_payload.dump();
}

std::string DroneBridge::build_lwt_payload() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    json conn_payload;
    conn_payload["header_id"]        = connection_msg_id_++;
    conn_payload["timestamp"]       = get_iso8601_timestamp();
    conn_payload["version"]         = cfg_.version;
    conn_payload["manufacturer"]    = cfg_.manufacturer;
    conn_payload["serial_number"]    = cfg_.serial_number;
    conn_payload["connection_state"] = "CONNECTIONBROKEN";
    return conn_payload.dump();
}

std::string DroneBridge::get_iso8601_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::gmtime(&now_c);
 
    std::ostringstream ss;
    ss << std::put_time(now_tm, "%Y-%m-%dT%H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << 'Z';
    return ss.str();
}


std::string DroneBridge::topic_prefix() const {
    return "drone/v2/" + this->cfg_.manufacturer + "/" + this->cfg_.serial_number;
}

const char* DroneBridge::copter_mode_name(uint32_t mode) {
    switch (mode) {
        case 0: return "STABILIZE";
        case 1: return "ACRO";
        case 2: return "ALT_HOLD";
        case 3: return "AUTO";
        case 4: return "GUIDED";
        case 5: return "LOITER";
        case 6: return "RTL";
        case 7: return "CIRCLE";
        case 9: return "LAND";
        case 11: return "DRIFT";
        case 13: return "SPORT";
        case 14: return "FLIP";
        case 15: return "AUTOTUNE";
        case 16: return "POSHOLD";
        case 17: return "BRAKE";
        case 18: return "THROW";
        case 19: return "AVOID_ADSB";
        case 20: return "GUIDED_NOGPS";
        case 21: return "SMART_RTL";
        case 22: return "FLOWHOLD";
        case 23: return "FOLLOW";
        case 24: return "ZIGZAG";
        case 25: return "SYSTEMID";
        case 26: return "AUTOROTATE";
        case 27: return "AUTO_RTL";
        default: return "UNKNOWN";
    }
}

void DroneBridge::mavlink_receive_loop() {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(cfg_.bind_address.c_str());
    addr.sin_port = htons(cfg_.udp_port);

    if (bind(sock, (const struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind failed\n");
        return;
    } else {
        printf("bind succeeded\n");
    }

    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t  status;

    while (running_) {
        ssize_t n = recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status) != 1)
                continue;

            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);

                    bool armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                    const char* mode = copter_mode_name(hb.custom_mode);

                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        armed_ = armed;
                        flight_mode_num_ = hb.custom_mode;
                        flight_mode_ = mode;
                        last_heartbeat_ = std::chrono::steady_clock::now();
                        drone_online_ = true;
                    }

                    // printf("throttle arm status=%s, mode=%s, (num=%u)\n",
                    //        armed ? "ARMED" : "DISARMED", mode, hb.custom_mode);
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS: {
                    mavlink_sys_status_t sys;
                    mavlink_msg_sys_status_decode(&msg, &sys);

                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        battery_pct_ = sys.battery_remaining;
                        voltage_mV_  = sys.voltage_battery;
                        current_cA_  = sys.current_battery;
                    }

                    // printf("battery=%d%%, voltage=%umV, current=%dcA\n",
                    //        sys.battery_remaining, sys.voltage_battery, sys.current_battery);
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                    mavlink_global_position_int_t pos;
                    mavlink_msg_global_position_int_decode(&msg, &pos);

                    double lat = pos.lat / 1e7;
                    double lon = pos.lon / 1e7;
                    double alt = pos.alt / 1000.0;
                    double vn = pos.vx / 100.0;
                    double ve = pos.vy / 100.0;
                    double vd = pos.vz / 100.0;

                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        lat_ = lat;
                        lon_ = lon;
                        alt_ = alt;
                        vn_ = vn;
                        ve_ = ve;
                        vd_ = vd;
                    }

                    // printf("lat=%.7f, lon=%.7f, alt=%.2fm, vN=%.2f, vE=%.2f, vD=%.2f\n",
                    //        lat, lon, alt, vn, ve, vd);
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE: {
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);

                    // printf("roll=%.4f, pitch=%.4f, yaw=%.4f\n",
                    //        att.roll, att.pitch, att.yaw);
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        roll_       = att.roll;       // rad
                        pitch_      = att.pitch;      // rad
                        yaw_        = att.yaw;        // rad
                        rollspeed_  = att.rollspeed;  // rad/s
                        pitchspeed_ = att.pitchspeed; // rad/s
                        yawspeed_   = att.yawspeed;   // rad/s
                    }
                    
                    break;
                }

                default:
                    break;
            }
        }
    }

    close(sock);
}

int main(int argc, char* argv[]) {
    std::string config_path = (argc > 1) ? argv[1] : "config.conf";
    BridgeConfig cfg = load_config(config_path);

    // no SA_RESTART so blocking calls are interrupted by the signal
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    DroneBridge bridge(cfg);

    
    bridge.run();
    bridge.shutdown();
    return 0;
}