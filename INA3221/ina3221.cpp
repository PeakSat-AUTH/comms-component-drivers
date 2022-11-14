#include "ina3221.hpp"

namespace INA3221 {

    uint16_t voltage_conversion(int32_t v, uint16_t scale, uint16_t shift) {
        return (v < 0) ?
               (~(std::abs(v) / scale << shift) + 1 & 0x7FFF) | 0x8000 :
               (v / scale << shift);
    }

    Error INA3221::write_register_field(Register address, uint16_t value, uint16_t mask, uint16_t shift) {
        auto[reg, err] = i2c_read(address);
        if (err != Error::NO_ERRORS) {
            return err;
        }
        uint16_t val = (reg & ~mask) | (value << shift);
        err = i2c_write(address, val);
        return err;
    }

    Error INA3221::setup() {
        uint16_t mode = (config.enableChannel1 << 14) | (config.enableChannel1 << 13) | (config.enableChannel1 << 12) |
                        ((uint16_t) config.averagingMode << 10) | ((uint16_t) config.busVoltageTime << 6) |
                        ((uint16_t) config.shuntVoltageTime << 3) | ((uint16_t) config.operatingMode);

        Error err;

        err = i2c_write(Register::CONFG, mode);

        if (err != Error::NO_ERRORS) { return err; }

        auto[criticalThreshold1, warningThreshold1] = config.threshold1;

        err = i2c_write(Register::CH1CA, voltage_conversion(criticalThreshold1, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::CH1WA, voltage_conversion(warningThreshold1, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        auto[criticalThreshold2, warningThreshold2] = config.threshold2;

        err = i2c_write(Register::CH2CA, voltage_conversion(criticalThreshold2, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::CH2WA, voltage_conversion(warningThreshold2, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        auto[criticalThreshold3, warningThreshold3] = config.threshold3;

        err = i2c_write(Register::CH3CA, voltage_conversion(criticalThreshold3, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::CH3WA, voltage_conversion(warningThreshold3, 40, 3));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::SHVLL, voltage_conversion(config.shuntVoltageSumLimit, 40, 1));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::PWRVU, voltage_conversion(config.powerValidUpper, 8000, 1));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::PWRVL, voltage_conversion(config.powerValidLower, 8000, 1));
        if (err != Error::NO_ERRORS) { return err; }

        err = i2c_write(Register::MASKE,
                        (config.summationChannelControl1 << 14) | (config.summationChannelControl2 << 13)
                        | (config.summationChannelControl3 << 12) | (config.enableWarnings << 11) |
                        (config.enableCritical << 10));
        return err;
    }

    void INA3221::handle_irq(void) {
        auto[value, err] = i2c_read(Register::MASKE);

        if (value & 0x0200){
            // Send critical alert of channel 1 to FDIR
        }
        if (value & 0x0100){
            // Send critical alert of channel 2 to FDIR
        }
        if (value & 0x0080){
            // Send critical alert of channel 3 to FDIR
        }
        if (value & 0x0040){
            // Send summation alert flag to FDIR
        }
        if (value & 0x0020){
            // Send warning alert of channel 1 to FDIR
        }
        if (value & 0x0010){
            // Send warning alert of channel 2 to FDIR
        }
        if (value & 0x0008){
            // Send warning alert of channel 3 to FDIR
        }
        if (value & 0x0004){
            //
        }
        if (value & 0x0002){
            //
        }
        if (value & 0x0001){
            uint16_t bus_voltage1 = i2c_read(Register::CH1BV).first;
            uint16_t bus_voltage2 = i2c_read(Register::CH2BV).first;
            uint16_t bus_voltage3 = i2c_read(Register::CH3BV).first;

            uint16_t shunt_voltage1 = i2c_read(Register::CH1SV).first;
            uint16_t shunt_voltage2 = i2c_read(Register::CH2SV).first;
            uint16_t shunt_voltage3 = i2c_read(Register::CH3SV).first;


            // Check if bus voltage is monitored
            if ((uint16_t) config.operatingMode & 0x10 || (uint16_t) config.operatingMode & 0x11) {
                busVoltage = std::make_tuple(
                        config.enableChannel1 ? NULL : bus_voltage1,
                        config.enableChannel2 ? NULL : bus_voltage2,
                        config.enableChannel3 ? NULL : bus_voltage3
                );
            }
            else {
                busVoltage = std::make_tuple(NULL, NULL, NULL);
            }

            // Check if shunt voltage is monitored
            if ((uint16_t) config.operatingMode & 0x01 || (uint16_t) config.operatingMode & 0x11) {
                busVoltage = std::make_tuple(
                        config.enableChannel1 ? NULL : shunt_voltage1,
                        config.enableChannel2 ? NULL : shunt_voltage2,
                        config.enableChannel3 ? NULL : shunt_voltage3
                );
            }
            else {
                shuntVoltage = std::make_tuple(NULL, NULL, NULL);
            }
        }
    }
}


int main() { return 0; }