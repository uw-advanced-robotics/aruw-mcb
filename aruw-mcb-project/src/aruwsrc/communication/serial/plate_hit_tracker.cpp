#include "plate_hit_tracker.hpp"



namespace aruwsrc::communication::serial
{
    PlateHitTracker::PlateHitTracker(tap::Drivers *drivers) : drivers(drivers) {}

    void PlateHitTracker::initalize()
    {
        dataTimestamp = -1;
        hitAngle_chassisRelative_radians = -1;
        hitAngle_worldRelative_radians =  -1;
        hitRecently = false;
        hitTimer.stop();
    }
    /**
     * 
     */
    void PlateHitTracker::update()
    {
        if (this->drivers->refSerial.getRobotData().receivedDps > lastDPS)
        {
            hitTimer.restart(HIT_EXPIRE_TIME);
            dataTimestamp = this->drivers->refSerial.getRobotData().robotDataReceivedTimestamp;
            hitRecently = true;
            lastDPS = this->drivers->refSerial.getRobotData().receivedDps;
            lastHitPlateID = static_cast<int>(this->drivers->refSerial.getRobotData().damagedArmorId);
        }
        hitRecently = !hitTimer.isExpired();
    }

    bool PlateHitTracker::isHitRecently()
    {
        return hitRecently;
    }


    PlateHitData PlateHitTracker::getLastHitData()
    {
        PlateHitData hitData;
        hitData.plateID = lastHitPlateID;
        hitData.hitAngleChassisRadians = lastHitPlateID * M_PI / 2;
        hitData.hitAngleWorldRadians = (lastHitPlateID * M_PI / 2) + modm::toRadian(this->drivers->mpu6500.getYaw()); 
        hitData.timestamp = dataTimestamp;
        return hitData;
    }
    
}