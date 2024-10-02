
#include "tap/drivers.hpp"

#include <tap/architecture/timeout.hpp>




namespace aruwsrc::communication::serial
{
    class PlateHitTracker {
        public:
            PlateHitTracker(tap::Drivers  *drivers);

            void initalize();
            void update();
            PlateHitData getRecentHitData();
            PlateHitData getLastHitData();
            bool isHitRecently();
        private:
            tap::Drivers *drivers;
            int dataTimestamp;
            float hitAngle_chassisRelative_radians;
            float hitAngle_worldRelative_radians;
            bool hitRecently;
            float lastDPS;
            int lastHitPlateID;
            float hitAngleChassisRadians;
            float hitAngleWorldRadians;
            tap::arch::MilliTimeout hitTimer;
            const int HIT_EXPIRE_TIME = 1000;
    };

    struct PlateHitData
    {
        int plateID;
        float hitAngleChassisRadians;
        float hitAngleWorldRadians;
        int timestamp;
    };

}