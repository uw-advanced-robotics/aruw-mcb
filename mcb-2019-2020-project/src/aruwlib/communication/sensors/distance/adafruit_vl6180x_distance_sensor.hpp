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
    static constexpr uint8_t DEVICE_ADDRESS = 0X29;

    static constexpr uint8_t VL6180X_REG_IDENTIFICATION_RESPONSE = 0xB4;

    enum class
    Register : uint16_t
    {
        IDENTIFICATION_MODEL_ID              = 0x000,
        SYSTEM__MODE_GPIO0                   = 0X010,
        SYSTEM__MODE_GPIO1                   = 0X011,
        SYSTEM_INTERRUPT_CONFIG_GPIO         = 0x014,
        SYSTEM_INTERRUPT_CLEAR               = 0x015,
        SYSTEM_FRESH_OUT_OF_RESET            = 0x016,
        SYSTEM_GROUPED_PARAMETER_HOLD        = 0X017,
        SYSRANGE_START                       = 0x018,
        SYSRANGE_THRESH_HIGH                 = 0X019,
        SYSRANGE_THRESH_LOW                  = 0X01A,
        SYSRANGE_INTERMEASUREMENT_PERIOD     = 0X01B,
        SYSRANGE_MAX_CONVERGENCE_TIME        = 0X01C,
        SYSRANGE_CROSSTALK_COMPENSATION_RATE = 0X01E,
        SYSRANGE__CROSSTALK_VALID_HEIGHT     = 0X021,
        SYSRANGE__EARLY_CONVERGENCE_ESTIMATE = 0x022,
        SYSRANGE__PART_TO_PART_RANGE_OFFSET  = 0x024,
        SYSRANGE__RANGE_IGNORE_VALID_HEIGHT  = 0x025,
        SYSRANGE__RANGE_IGNORE_THRESHOLD     = 0x026,
        SYSRANGE__MAX_AMBIENT_LEVEL_MULT     = 0x02C,
        SYSRANGE__RANGE_CHECK_ENABLES        = 0x02D,
        SYSRANGE__VHV_RECALIBRATE            = 0x02E,
        SYSRANGE__VHV_REPEAT_RATE            = 0x031,
        SYSALS__START                        = 0x038,
        SYSALS__THRESH_HIGH                  = 0x03A,
        SYSALS__THRESH_LOW                   = 0x03C,
        SYSALS__INTERMEASUREMENT_PERIOD      = 0x03E,
        SYSALS__ANALOGUE_GAIN                = 0x03F,
        SYSALS__INTEGRATION_PERIOD           = 0x040,
        RESULT__RANGE_STATUS                 = 0x04D,
        RESULT__ALS_STATUS                   = 0x04E,
        RESULT__INTERRUPT_STATUS_GPIO        = 0x04F,
        RESULT__ALS_VAL                      = 0x050,
        RESULT__RANGE_VAL                    = 0x062,
        RESULT__RANGE_RAW                    = 0x064,
        RESULT__RANGE_RETURN_RATE            = 0x066,
        RESULT__RANGE_REFERENCE_RATE         = 0x068,
        RESULT__RANGE_RETURN_SIGNAL_COUNT    = 0x06C,
        RESULT__RANGE_REFERENCE_SIGNAL_COUNT = 0x070,
        RESULT__RANGE_RETURN_AMB_COUNT       = 0x074,
        RESULT__RANGE_REFERENCE_AMB_COUNT    = 0x078,
        RESULT__RANGE_RETURN_CONV_TIME       = 0x07C,
        RESULT__RANGE_REFERENCE_CONV_TIME    = 0x080,
        READOUT__AVERAGING_SAMPLE_PERIOD     = 0x10A,
        FIRMWARE__BOOTUP                     = 0x119,
        FIRMWARE__RESULT_SCALER              = 0x120,
        I2C_SLAVE__DEVICE_ADDRESS            = 0x212,
        INTERLEAVED_MODE__ENABLE             = 0x2A3,
    };

    enum class
    ALSGain : uint8_t
    {
        VL6180X_ALS_GAIN_1    = 0x06,  ///< 1x gain
        VL6180X_ALS_GAIN_1_25 = 0x05,  ///< 1.25x gain
        VL6180X_ALS_GAIN_1_67 = 0x04,  ///< 1.67x gain
        VL6180X_ALS_GAIN_2_5  = 0x03,  ///< 2.5x gain
        VL6180X_ALS_GAIN_5    = 0x02,  ///< 5x gain
        VL6180X_ALS_GAIN_10   = 0x01,  ///< 10x gain
        VL6180X_ALS_GAIN_20   = 0x00,  ///< 20x gain
        VL6180X_ALS_GAIN_40   = 0x07   ///< 40x gain
    };
};

/**
 * @attention The vl6180 expects 16 bit register requests, so when reading and writing to registers,
 *            the configuration is set to send the register as 16 bit (2 eight bit values).
 */
template < class I2cMaster >
class Vl6810xDistanceSensor : public DistanceSensor, public modm::I2cDevice<I2cMaster, 4>, public Vl6810xConstants, public modm::pt::Protothread
{
 public:
    Vl6810xDistanceSensor(uint32_t vl6180XCommunicationTimeoutPeriod = DEFAULT_COMMUNICATION_TIMEOUT) :
            DistanceSensor(0.0f, 100.0f),
            modm::I2cDevice<I2cMaster, 4>(DEVICE_ADDRESS),
            communicateTimeout(vl6180XCommunicationTimeoutPeriod)
    {}

    void init() override
    {}

    float read() override
    {
        return range;
    }

    bool validReading() override
    {
        return false;
    }

    bool update()
    {
        PT_BEGIN();

        while (true)
        {
            if (currState == VL6180XInitializationState::DEVICE_NOT_STARTED)
            {
                PT_CALL( readRegister(Register::IDENTIFICATION_MODEL_ID, data) );
                if (data[0] == VL6180X_REG_IDENTIFICATION_RESPONSE)
                {
                    currState = VL6180XInitializationState::CONFIGURING_DEVICE;
                }
                else
                {
                    PT_WAIT_UNTIL(communicateTimeout.execute());
                }
            }
            else if (currState == VL6180XInitializationState::CONFIGURING_DEVICE)
            {
                if (PT_CALL(loadSettings()))
                {
                    currState = VL6180XInitializationState::RECEIVING_DATA;
                }
                else
                {
                    PT_WAIT_UNTIL(communicateTimeout.execute());
                }
            }
            else if (currState == VL6180XInitializationState::RECEIVING_DATA)
            {
                do
                {
                    PT_CALL( readRegister(Register::RESULT__RANGE_STATUS, data) );
                } while (!(data[0] & 0x1));

                PT_CALL( writeRegister(Register::SYSRANGE_START, 0x1) );

                do
                {
                    PT_CALL( readRegister(Register::RESULT__INTERRUPT_STATUS_GPIO, data) );
                } while (!(data[0] & 0x04));

                // range in mm
                PT_CALL( readRegister(Register::RESULT__RANGE_VAL, data) );
                range = data[0];

                // clear interrupt
                PT_CALL( writeRegister(Register::SYSTEM_INTERRUPT_CLEAR, 0x07) );
                
                // readRange();
                PT_WAIT_UNTIL(communicateTimeout.execute());
            }
        }

        PT_END();
    }

 private:
    static constexpr uint32_t DEFAULT_COMMUNICATION_TIMEOUT = 5;

    enum class VL6180XInitializationState
    {
        DEVICE_NOT_STARTED,
        CONFIGURING_DEVICE,
        RECEIVING_DATA
    };

    VL6180XInitializationState currState = VL6180XInitializationState::DEVICE_NOT_STARTED;

    uint8_t buffer[4];

    uint8_t data[10];

    uint8_t prev_reg;

    uint8_t range;

    modm::ShortPeriodicTimer communicateTimeout;

    modm::ResumableResult<bool>
    begin()
    {
        RF_BEGIN();
        RF_END_RETURN(true);
    }

    inline modm::ResumableResult<bool>
    readRegister(Register reg, uint8_t *output, size_t length = 1)
    {
        RF_BEGIN();

        buffer[0] = uint16_t(reg) >> 8;
        buffer[1] = uint8_t(reg);

        this->transaction.configureWriteRead(buffer, 2, output, length);
        RF_END_RETURN_CALL( this->runTransaction() );
    }

    inline modm::ResumableResult<bool>
    writeRegister(Register reg, uint8_t output)
    {
        RF_BEGIN();
        buffer[0] = uint16_t(reg) >> 8;
        buffer[1] = uint8_t(reg);
        buffer[2] = output;
        this->transaction.configureWrite(buffer, 3);
        RF_END_RETURN_CALL( this->runTransaction() );
    }

    modm::ResumableResult<bool> loadSettings()
    {
        // Set to insure the device does not use any of the parameters midway through
        // setting update
        writeRegister(Register::SYSTEM_GROUPED_PARAMETER_HOLD, 0x01);

        // enables GPIO Interrupt output, enables polling for 'new sample ready when
        // measurement completes
        writeRegister(Register::SYSTEM__MODE_GPIO0, 0b00010000);

        // enable system interrupt trigger when a new sample is ready for both als and range
        writeRegister(Register::SYSTEM_INTERRUPT_CONFIG_GPIO, 0b00100100);

        return writeRegister(Register::SYSTEM_GROUPED_PARAMETER_HOLD, 0x00);


        // Recommended : Public registers - See data sheet for more detail
        // writeRegister(0x0011, 0x10);       // Enables polling for 'New Sample ready'
        //                             // when measurement completes
        // writeRegister(0x010a, 0x30);       // Set the averaging sample period
        //                             // (compromise between lower noise and
        //                             // increased execution time)
        // writeRegister(0x003f, 0x46);       // Sets the light and dark gain (upper
        //                             // nibble). Dark gain should not be
        //                             // changed.
        // writeRegister(0x0031, 0xFF);       // sets the # of range measurements after
        //                             // which auto calibration of system is
        //                             // performed
        // writeRegister(0x0040, 0x63);       // Set ALS integration time to 100ms
        // writeRegister(0x002e, 0x01);       // perform a single temperature calibration
        //                             // of the ranging sensor

        // // Optional: Public registers - See data sheet for more detail
        // writeRegister(0x001b, 0x09);       // Set default ranging inter-measurement
        //                             // period to 100ms
        // writeRegister(0x003e, 0x31);       // Set default ALS inter-measurement period
        //                             // to 500ms
        // writeRegister(0x0014, 0x24);       // Configures interrupt on 'New Sample
        //                             // Ready threshold event'

        // use this to check for a reset condition, default is 1
        writeRegister(Register::SYSTEM_FRESH_OUT_OF_RESET, 0x0);
    }
};

}  // namespace sensors

}  // namespace aruwlib

#endif
