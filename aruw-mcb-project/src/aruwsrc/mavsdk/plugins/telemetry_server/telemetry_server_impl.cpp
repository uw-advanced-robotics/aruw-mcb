#include "telemetry_server_impl.h"
#include "unused.h"

namespace mavsdk {

TelemetryServerImpl::TelemetryServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
    _start_time = std::chrono::steady_clock::now();
}

TelemetryServerImpl::~TelemetryServerImpl()
{
    _server_component_impl->unregister_plugin(this);
    std::unique_lock<std::mutex> lock(_interval_mutex);
    for (const auto& request : _interval_requests) {
        _server_component_impl->remove_call_every(request.cookie);
    }
}

void TelemetryServerImpl::init()
{
    // Handle SET_MESSAGE_INTERVAL
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            std::lock_guard<std::mutex> lock(_interval_mutex);
            uint32_t msgid = static_cast<uint32_t>(command.params.param1);
            // Set interval to 1hz if 0 (default rate)
            uint32_t interval_ms =
                command.params.param2 == 0 ?
                    1000 :
                    static_cast<uint32_t>(static_cast<double>(command.params.param2) * 1E-3);
            LogDebug() << "Setting interval for msg id: " << std::to_string(msgid)
                       << " interval_ms:" << std::to_string(interval_ms);
            auto found = std::find_if(
                _interval_requests.begin(),
                _interval_requests.end(),
                [msgid](const RequestMsgInterval& item) { return item.msg_id == msgid; });

            if (found == _interval_requests.end() && command.params.param2 != -1) {
                // If not found interval already, add a new one
                _interval_requests.push_back({msgid, interval_ms, nullptr});
                _server_component_impl->add_call_every(
                    [this, msgid]() {
                        std::lock_guard<std::mutex> lock_interval(_interval_mutex);
                        if (_msg_cache.find(msgid) != _msg_cache.end()) {
                            // Publish if callback exists :)
                            _server_component_impl->send_message(_msg_cache.at(msgid));
                        }
                    },
                    static_cast<double>(interval_ms) * 1E-3,
                    &_interval_requests.back().cookie);
            } else {
                if (command.params.param2 == -1) {
                    // Deregister with -1 interval
                    _server_component_impl->remove_call_every(found->cookie);
                    _interval_requests.erase(found);
                } else {
                    // Update Interval
                    found->interval = interval_ms;
                    _server_component_impl->change_call_every(
                        static_cast<double>(interval_ms) * 1E-3, found->cookie);
                }
            }

            return _server_component_impl->make_command_ack_message(
                command, MAV_RESULT::MAV_RESULT_ACCEPTED);
        },
        this);
}

void TelemetryServerImpl::deinit() {}

TelemetryServer::Result TelemetryServerImpl::publish_position(
    TelemetryServer::Position position,
    TelemetryServer::VelocityNed velocity_ned,
    TelemetryServer::Heading heading)
{
    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        get_boot_time_ms(),
        static_cast<int32_t>(position.latitude_deg * 1E7),
        static_cast<int32_t>(position.longitude_deg * 1E7),
        static_cast<int32_t>(static_cast<double>(position.absolute_altitude_m) * 1E3),
        static_cast<int32_t>(static_cast<double>(position.relative_altitude_m) * 1E3),
        static_cast<int16_t>(static_cast<double>(velocity_ned.north_m_s) * 1E2),
        static_cast<int16_t>(static_cast<double>(velocity_ned.east_m_s) * 1E2),
        static_cast<int16_t>(static_cast<double>(velocity_ned.down_m_s) * 1E2),
        static_cast<uint16_t>(static_cast<double>(heading.heading_deg) * 1E2));

    add_msg_cache(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result TelemetryServerImpl::publish_home(TelemetryServer::Position home)
{
    mavlink_message_t msg;
    const float q[4] = {};
    mavlink_msg_home_position_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        static_cast<int32_t>(home.latitude_deg * 1E7),
        static_cast<int32_t>(home.longitude_deg * 1E7),
        static_cast<int32_t>(static_cast<double>(home.absolute_altitude_m) * 1E-3),
        0, // Local X
        0, // Local Y
        0, // Local Z
        q, // surface normal transform
        NAN, // approach x
        NAN, // approach y
        NAN, // approach z
        get_boot_time_ms() // TO-DO: System boot
    );

    add_msg_cache(MAVLINK_MSG_ID_HOME_POSITION, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result TelemetryServerImpl::publish_raw_gps(
    TelemetryServer::RawGps raw_gps, TelemetryServer::GpsInfo gps_info)
{
    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        raw_gps.timestamp_us,
        static_cast<uint8_t>(gps_info.fix_type),
        static_cast<int32_t>(raw_gps.latitude_deg * 1E7),
        static_cast<int32_t>(raw_gps.longitude_deg * 1E7),
        static_cast<int32_t>(static_cast<double>(raw_gps.absolute_altitude_m) * 1E3),
        static_cast<uint16_t>(static_cast<double>(raw_gps.hdop) * 1E2),
        static_cast<uint16_t>(static_cast<double>(raw_gps.vdop) * 1E2),
        static_cast<uint16_t>(static_cast<double>(raw_gps.velocity_m_s) * 1E2),
        static_cast<uint16_t>(static_cast<double>(raw_gps.cog_deg) * 1E2),
        static_cast<uint8_t>(gps_info.num_satellites),
        static_cast<int32_t>(static_cast<double>(raw_gps.altitude_ellipsoid_m) * 1E3),
        static_cast<uint32_t>(static_cast<double>(raw_gps.horizontal_uncertainty_m) * 1E3),
        static_cast<uint32_t>(static_cast<double>(raw_gps.vertical_uncertainty_m) * 1E3),
        static_cast<uint32_t>(static_cast<double>(raw_gps.velocity_uncertainty_m_s) * 1E3),
        static_cast<uint32_t>(static_cast<double>(raw_gps.heading_uncertainty_deg) * 1E5),
        static_cast<uint16_t>(static_cast<double>(raw_gps.yaw_deg) * 1E2));

    add_msg_cache(MAVLINK_MSG_ID_GPS_RAW_INT, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result TelemetryServerImpl::publish_battery(TelemetryServer::Battery battery)
{
    mavlink_message_t msg;

    uint16_t voltages[10] = {0};
    uint16_t voltages_ext[4] = {0};
    voltages[0] = static_cast<uint16_t>(static_cast<double>(battery.voltage_v) * 1E3);

    mavlink_msg_battery_status_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        0,
        MAV_BATTERY_FUNCTION_ALL,
        MAV_BATTERY_TYPE_LIPO,
        INT16_MAX,
        voltages,
        -1, // TODO publish all battery data
        -1,
        -1,
        static_cast<uint16_t>(static_cast<double>(battery.remaining_percent) * 1E2),
        0,
        MAV_BATTERY_CHARGE_STATE_UNDEFINED,
        voltages_ext,
        MAV_BATTERY_MODE_UNKNOWN,
        0);

    add_msg_cache(MAVLINK_MSG_ID_BATTERY_STATUS, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result
TelemetryServerImpl::publish_status_text(TelemetryServer::StatusText status_text)
{
    mavlink_message_t msg;

    int type = MAV_SEVERITY_INFO;
    switch (status_text.type) {
        case TelemetryServer::StatusTextType::Emergency:
            type = MAV_SEVERITY_EMERGENCY;
            break;
        case TelemetryServer::StatusTextType::Alert:
            type = MAV_SEVERITY_ALERT;
            break;
        case TelemetryServer::StatusTextType::Critical:
            type = MAV_SEVERITY_CRITICAL;
            break;
        case TelemetryServer::StatusTextType::Error:
            type = MAV_SEVERITY_ERROR;
            break;
        case TelemetryServer::StatusTextType::Warning:
            type = MAV_SEVERITY_WARNING;
            break;
        case TelemetryServer::StatusTextType::Notice:
            type = MAV_SEVERITY_NOTICE;
            break;
        case TelemetryServer::StatusTextType::Info:
            type = MAV_SEVERITY_INFO;
            break;
        case TelemetryServer::StatusTextType::Debug:
            type = MAV_SEVERITY_DEBUG;
            break;
        default:
            LogWarn() << "Unknown StatusText severity";
            type = MAV_SEVERITY_INFO;
            break;
    }

    // Prevent memcpy in mavlink function to read outside of allocated data.
    status_text.text.resize(sizeof(mavlink_statustext_t::text));

    mavlink_msg_statustext_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        type,
        status_text.text.data(),
        0,
        0);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result TelemetryServerImpl::publish_odometry(TelemetryServer::Odometry odometry)
{
    UNUSED(odometry);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_position_velocity_ned(
    TelemetryServer::PositionVelocityNed position_velocity_ned)
{
    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        get_boot_time_ms(),
        position_velocity_ned.position.north_m,
        position_velocity_ned.position.east_m,
        position_velocity_ned.position.down_m,
        position_velocity_ned.velocity.north_m_s,
        position_velocity_ned.velocity.east_m_s,
        position_velocity_ned.velocity.down_m_s);

    add_msg_cache(MAVLINK_MSG_ID_LOCAL_POSITION_NED, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

TelemetryServer::Result
TelemetryServerImpl::publish_ground_truth(TelemetryServer::GroundTruth ground_truth)
{
    UNUSED(ground_truth);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_imu(TelemetryServer::Imu imu)
{
    UNUSED(imu);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_scaled_imu(TelemetryServer::Imu imu)
{
    UNUSED(imu);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_raw_imu(TelemetryServer::Imu imu)
{
    UNUSED(imu);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_unix_epoch_time(uint64_t time_us)
{
    UNUSED(time_us);

    // TODO :)
    return {};
}

TelemetryServer::Result TelemetryServerImpl::publish_sys_status(
    TelemetryServer::Battery battery,
    bool rc_receiver_status,
    bool gyro_status,
    bool accel_status,
    bool mag_status,
    bool gps_status)
{
    int32_t sensors = 0;

    if (rc_receiver_status) {
        sensors |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (gyro_status) {
        sensors |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (accel_status) {
        sensors |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if (mag_status) {
        sensors |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps_status) {
        sensors |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        sensors,
        sensors,
        sensors,
        0,
        static_cast<uint16_t>(static_cast<double>(battery.voltage_v) * 1E3),
        -1,
        static_cast<uint16_t>(static_cast<double>(battery.remaining_percent) * 1E2),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0);

    add_msg_cache(MAVLINK_MSG_ID_SYS_STATUS, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

uint8_t to_mav_vtol_state(TelemetryServer::VtolState vtol_state)
{
    switch (vtol_state) {
        case TelemetryServer::VtolState::Undefined:
            return MAV_VTOL_STATE_UNDEFINED;
        case TelemetryServer::VtolState::TransitionToFw:
            return MAV_VTOL_STATE_TRANSITION_TO_FW;
        case TelemetryServer::VtolState::TransitionToMc:
            return MAV_VTOL_STATE_TRANSITION_TO_MC;
        case TelemetryServer::VtolState::Mc:
            return MAV_VTOL_STATE_MC;
        case TelemetryServer::VtolState::Fw:
            return MAV_VTOL_STATE_FW;
        default:
            return MAV_VTOL_STATE_UNDEFINED;
    }
}

uint8_t to_mav_landed_state(TelemetryServer::LandedState landed_state)
{
    switch (landed_state) {
        case TelemetryServer::LandedState::InAir:
            return MAV_LANDED_STATE_IN_AIR;
        case TelemetryServer::LandedState::TakingOff:
            return MAV_LANDED_STATE_TAKEOFF;
        case TelemetryServer::LandedState::Landing:
            return MAV_LANDED_STATE_LANDING;
        case TelemetryServer::LandedState::OnGround:
            return MAV_LANDED_STATE_ON_GROUND;
        default:
            return MAV_LANDED_STATE_UNDEFINED;
    }
}

TelemetryServer::Result TelemetryServerImpl::publish_extended_sys_state(
    TelemetryServer::VtolState vtol_state, TelemetryServer::LandedState landed_state)
{
    mavlink_message_t msg;
    mavlink_msg_extended_sys_state_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        to_mav_vtol_state(vtol_state),
        to_mav_landed_state(landed_state));

    add_msg_cache(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, msg);

    return _server_component_impl->send_message(msg) ? TelemetryServer::Result::Success :
                                                       TelemetryServer::Result::Unsupported;
}

void TelemetryServerImpl::add_msg_cache(uint64_t id, mavlink_message_t& msg)
{
    std::unique_lock<std::mutex> lock(_interval_mutex);
    _msg_cache.insert_or_assign(id, msg);
}

} // namespace mavsdk
