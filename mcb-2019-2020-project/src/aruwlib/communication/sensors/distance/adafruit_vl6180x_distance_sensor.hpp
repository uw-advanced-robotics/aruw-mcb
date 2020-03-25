#ifndef __ADAFRUIT_VL6180X_DISTANCE_SENSOR_HPP__
#define __ADAFRUIT_VL6180X_DISTANCE_SENSOR_HPP__

#include <modm/platform/i2c/i2c_master_2.hpp>
#include <modm/architecture/interface/register.hpp>
#include <modm/processing/resumable.hpp>

#include "distance_sensor.hpp"

namespace aruwlib
{

namespace sensors
{

struct Vl6810xConstants
{
    enum class
    Register : uint8_t
    {
        ///! Device model identification number
        VL6180X_REG_IDENTIFICATION_MODEL_ID   = 0x000,
        ///! Interrupt configuration
        VL6180X_REG_SYSTEM_INTERRUPT_CONFIG      = 0x014,
        ///! Interrupt clear bits
        VL6180X_REG_SYSTEM_INTERRUPT_CLEAR       = 0x015,
        ///! Fresh out of reset bit
        VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET    = 0x016,
        ///! Trigger Ranging
        VL6180X_REG_SYSRANGE_START               = 0x018,
        ///! Trigger Lux Reading
        VL6180X_REG_SYSALS_START                 = 0x038,
        ///! Lux reading gain
        VL6180X_REG_SYSALS_ANALOGUE_GAIN         = 0x03F,
        ///! Integration period for ALS mode, high byte
        VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI = 0x040,
        ///! Integration period for ALS mode, low byte
        VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO = 0x041,
        ///! Specific error codes
        VL6180X_REG_RESULT_RANGE_STATUS          = 0x04d,
        ///! Interrupt status
        VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x04f,
        ///! Light reading value
        VL6180X_REG_RESULT_ALS_VAL               = 0x050,
        ///! Ranging reading value
        VL6180X_REG_RESULT_RANGE_VAL             = 0x062
    };
};

template < class I2cMaster >
class Vl6810xDistanceSensor : public DistanceSensor, public modm::I2cDevice<I2cMaster, 4>, public Vl6810xConstants
{
 public:
    Vl6810xDistanceSensor() :
            DistanceSensor(0.0f, 100.0f), modm::I2cDevice<I2cMaster, 4>(DEVICE_ADDRESS)
    {}

    void init() override
    {

    }

    float read() override
    {
        return 0.0f;
    }

    bool validReading() override
    {
        return false;
    }

    modm::ResumableResult<bool>
    readRegister(Register reg, uint8_t *output, size_t length=1)
    {
        RF_BEGIN();
        // RF_CALL(setPageId(reg));

        buffer[0] = uint8_t(reg);
        this->transaction.configureWriteRead(buffer, 1, output, length);
        RF_END_RETURN_CALL( this->runTransaction() );
    }

 private:
    static constexpr uint8_t DEVICE_ADDRESS = 0X29;

	uint8_t buffer[3];

	uint8_t prev_reg;

	// inline modm::ResumableResult<bool>
	// setPageId(Register regi)
	// {
	// 	const uint8_t reg = uint8_t(regi);
	// 	RF_BEGIN();

	// 	if ((reg ^ prev_reg) & 0x80) {
	// 		buffer[0] = uint8_t(Register::PAGE_ID);
	// 		buffer[1] = reg >> 7;
	// 		this->transaction.configureWrite(buffer, 2);
	// 		buffer[2] = RF_CALL( this->runTransaction() );
	// 		if (buffer[2]) prev_reg = reg;
	// 		RF_RETURN(buffer[2]);
	// 	}

	// 	RF_END_RETURN(true);
	// }
};

}  // namespace sensors

}  // namespace aruwlib


#endif
