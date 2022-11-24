#include "geofence_impl.h"
#include "log.h"
#include <cmath>

namespace mavsdk {

GeofenceImpl::GeofenceImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

GeofenceImpl::GeofenceImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _parent->register_plugin(this);
}

GeofenceImpl::~GeofenceImpl()
{
    _parent->unregister_plugin(this);
}

void GeofenceImpl::init() {}

void GeofenceImpl::deinit() {}

void GeofenceImpl::enable() {}

void GeofenceImpl::disable() {}

Geofence::Result GeofenceImpl::upload_geofence(const std::vector<Geofence::Polygon>& polygons)
{
    auto prom = std::promise<Geofence::Result>();
    auto fut = prom.get_future();

    upload_geofence_async(polygons, [&prom](Geofence::Result result) { prom.set_value(result); });
    return fut.get();
}

void GeofenceImpl::upload_geofence_async(
    const std::vector<Geofence::Polygon>& polygons, const Geofence::ResultCallback& callback)
{
    // We can just create these items on the stack because they get copied
    // later in the MavlinkMissionTransfer constructor.
    const auto items = assemble_items(polygons);

    _parent->mission_transfer().upload_items_async(
        MAV_MISSION_TYPE_FENCE, items, [this, callback](MavlinkMissionTransfer::Result result) {
            auto converted_result = convert_result(result);
            _parent->call_user_callback(
                [callback, converted_result]() { callback(converted_result); });
        });
}

Geofence::Result GeofenceImpl::clear_geofence()
{
    auto prom = std::promise<Geofence::Result>();
    auto fut = prom.get_future();

    clear_geofence_async([&prom](Geofence::Result result) { prom.set_value(result); });
    return fut.get();
}

void GeofenceImpl::clear_geofence_async(const Geofence::ResultCallback& callback)
{
    _parent->mission_transfer().clear_items_async(
        MAV_MISSION_TYPE_FENCE, [this, callback](MavlinkMissionTransfer::Result result) {
            auto converted_result = convert_result(result);
            _parent->call_user_callback([callback, converted_result]() {
                if (callback) {
                    callback(converted_result);
                }
            });
        });
}

std::vector<MavlinkMissionTransfer::ItemInt>
GeofenceImpl::assemble_items(const std::vector<Geofence::Polygon>& polygons)
{
    std::vector<MavlinkMissionTransfer::ItemInt> items;

    uint16_t sequence = 0;
    for (auto& polygon : polygons) {
        uint16_t command;
        switch (polygon.fence_type) {
            case Geofence::Polygon::FenceType::Inclusion:
                command = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
                break;
            case Geofence::Polygon::FenceType::Exclusion:
                command = MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
                break;
            default:
                LogErr() << "Unknown type";
                continue;
        }

        for (auto& point : polygon.points) {
            // FIXME: check if these two  make sense.
            const uint8_t current = (sequence == 0 ? 1 : 0);
            const uint8_t autocontinue = 0;
            const float param1 = float(polygon.points.size());

            items.push_back(MavlinkMissionTransfer::ItemInt{
                sequence,
                MAV_FRAME_GLOBAL_INT,
                command,
                current,
                autocontinue,
                param1,
                0.0f,
                0.0f,
                0.0f,
                int32_t(std::round(point.latitude_deg * 1e7)),
                int32_t(std::round(point.longitude_deg * 1e7)),
                0.0f,
                MAV_MISSION_TYPE_FENCE});
            ++sequence;
        }
    }
    return items;
}

Geofence::Result GeofenceImpl::convert_result(MavlinkMissionTransfer::Result result)
{
    switch (result) {
        case MavlinkMissionTransfer::Result::Success:
            return Geofence::Result::Success;
        case MavlinkMissionTransfer::Result::ConnectionError:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::Denied:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::TooManyMissionItems:
            return Geofence::Result::TooManyGeofenceItems;
        case MavlinkMissionTransfer::Result::Timeout:
            return Geofence::Result::Timeout;
        case MavlinkMissionTransfer::Result::Unsupported:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::UnsupportedFrame:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::NoMissionAvailable:
            return Geofence::Result::InvalidArgument; // FIXME
        case MavlinkMissionTransfer::Result::Cancelled:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::MissionTypeNotConsistent:
            return Geofence::Result::InvalidArgument; // FIXME
        case MavlinkMissionTransfer::Result::InvalidSequence:
            return Geofence::Result::InvalidArgument; // FIXME
        case MavlinkMissionTransfer::Result::CurrentInvalid:
            return Geofence::Result::InvalidArgument; // FIXME
        case MavlinkMissionTransfer::Result::ProtocolError:
            return Geofence::Result::Error; // FIXME
        case MavlinkMissionTransfer::Result::InvalidParam:
            return Geofence::Result::InvalidArgument; // FIXME
        case MavlinkMissionTransfer::Result::IntMessagesNotSupported:
            return Geofence::Result::Error; // FIXME
        default:
            return Geofence::Result::Unknown;
    }
}

} // namespace mavsdk
