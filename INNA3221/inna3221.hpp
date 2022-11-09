#ifndef COMPONENT_DRIVERS_INNA3221_HPP
#define COMPONENT_DRIVERS_INNA3221_HPP

#include <cstdint>
#include <valarray>
#include <type_traits>

namespace INNA3221{
    typedef std::pair<uint32_t, uint32_t> VoltageThreshold;

    enum class Register{
        CONFG   = 0x00, /// Configuration
        CH1SV   = 0x01, /// Channel 1 Shunt Voltage
        CH1BV   = 0x02, /// Chanel 1 Bus Voltage
        CH2SV   = 0x03, /// Channel 2 Shunt Voltage
        CH2BV   = 0x04, /// Chanel 2 Bus Voltage
        CH3SV   = 0x05, /// Channel 3 Shunt Voltage
        CH3BV   = 0x06, /// Chanel 3 Bus Voltage
        CH1CA   = 0x07, /// Chanel 1 Critical Alert Limit
        CH1WA   = 0x08, /// Chanel 1 Warning Alert Limit
        CH2CA   = 0x09, /// Chanel 2 Critical Alert Limit
        CH2WA   = 0x0A, /// Chanel 2 Warning Alert Limit
        CH3CA   = 0x0B, /// Chanel 3 Critical Alert Limit
        CH3WA   = 0x0C, /// Chanel 3 Warning Alert Limit
        SHVOL   = 0x0D, /// Shunt-Voltage Sum
        SHVLL   = 0x0E, /// Shunt-Voltage Sum Limit
        MASKE   = 0x0F, /// Mask-Enable
        PWRVU   = 0x10, /// Power-Valid Upper Limit
        PWRVL   = 0x11, /// Power-Valid Lower Limit
        MFRID   = 0xFE, /// Manufaturer ID
        DIEID   = 0xFF, /// Die ID
    };

    enum class Alert{
        CRITICAL = 0,
        WARNING,
        POWER_VALID,
        TIMING_CONTROL
    };

    enum class Error {
        NO_ERRORS = 0,
        I2C_FAIlURE,
    };

    /// The number of voltage samples that are averaged together
    enum class AveragingMode {
        AVG_1       = 0,
        AVG_4       = 1,
        AVG_16      = 2,
        AVG_64      = 3,
        AVG_128     = 4,
        AVG_256     = 5,
        AVG_512     = 6,
        AVG_1024    = 7,
    };

    enum class VoltageTime {
        V_140_MS   = 0, /// 140 μs
        V_204_MS   = 1, /// 240 μs
        V_332_MS   = 2, /// 322 μs
        V_588_MS   = 3, /// 588 μs
        V_1_1_MS   = 4, /// 1.1 μs
        V_2_116_MS = 5, /// 2.116 μs
        V_4_156_MS = 6, /// 4.156 μs
        V_8_244_MS = 7, /// 8.244 μs
    };

    enum class OperatingMode{
        POWER_DOWN = 0,               /// Power-down
        SHUNT_VOLTAGE_SS = 1,         /// Shunt Voltage, Single-Shot
        BUS_VOLTAGE_SS = 2,           /// Bus Voltage, Single-Shot
        SHUNT_BUS_VOLTAGE_SS = 3,     /// Shunt & Bus Voltage, Single-Shot
        POWER_DOWN_REND = 4,          /// Power-down
        SHUNT_VOLTAGE_CONT = 1,       /// Shunt Voltage, Continuous
        BUS_VOLTAGE_CONT = 2,         /// Bus Voltage, Continuous
        SHUNT_BUS_VOLTAGE_CONT = 3,   /// Shunt & Bus Voltage, Continuous

    };

    struct INNA3221Config{

        /// Determines whether channel 1 is enabled
        bool enable_channel1 = true;

        /// Determines whether channel 2 is enabled
        bool enable_channel2 = true;

        /// Determines whether channel 3 is enabled
        bool enable_channel3 = true;

        /// The number of voltage samples that are averaged together
        AveragingMode averagingMode = AveragingMode::AVG_4;

        /**
          * Time of bus voltage measurement conversion.
          * This value should be selected according to the time requirements of the application.
          * Note that it applies to all channels
         */
        VoltageTime bus_voltage_time = VoltageTime::V_1_1_MS;

        /**
          * Time of shunt voltage measurement conversion.
          * This value should be selected according to the time requirements of the application.
          * Note that it applies to all channels
         */
        VoltageTime shunt_voltage_time = VoltageTime::V_1_1_MS;

        /**
         * Select mode operation of INNA3211. The main three modes are:
         *  - Power down: Turns off the current drawn to reduce power consumption. Switching from power-down
         *  mode takes approximately 40 μs. I2C communication is still enabled while in this mode
         *  - Single shot: Measurements are taken only whenever this register is written and set to single shot mode
         *  (there's no need to switch the value from a previous different state).
         *  - Continuous: Measurements are constantly taken periodically until the mode is switched to either
         *  single-shot or power down
         *
         *  This register also controls whether this is applies only to shunt voltage, bus voltage
         *  or both.
         */
        OperatingMode operating_mode = OperatingMode::POWER_DOWN;

        /// Shunt voltage threshold for critical and warning alert for channel1 [μV]
        VoltageThreshold threshold1;

        /// Shunt voltage threshold for critical and warning alert for channel2 [μV]
        VoltageThreshold threshold2;

        /// Shunt voltage threshold for critical and warning alert for channel3 [μV]
        VoltageThreshold threshold3;

        /// Shunt voltage sum limit [μV]
        uint32_t shunt_voltage_sum_limit;

        /// Upper limit of power-valid [μV]
        uint32_t power_valid_upper;

        /// Lower limit of power-valid [μV]
        uint32_t power_valid_lower;
    };

    class INNA3221 {
    public:
        INNA3221(const INNA3221Config&& config, Error &err):
                config(std::move(config)) {
            setup(err);
        };

        ~INNA3221(){};

    private:
        __attribute__ ((__weak__)) void i2c_write(Register address, uint16_t value, Error &err);
        __attribute__ ((__weak__)) uint16_t i2c_read(Register address, Error &err){
            return 0;
        };

        void setup(Error &err);

        INNA3221Config config;
    };
}


#endif //COMPONENT_DRIVERS_INNA3221_HPP
