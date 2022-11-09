#include "inna3221.hpp"

namespace INNA3221 {

    uint16_t voltage_conversion(int32_t v, uint16_t scale, uint16_t shift){
        return (v < 0) ?
        (~(std::abs(v) / scale << shift) + 1 & 0x7FFF) | 0x8000:
        (v / scale << shift);
    }

    void INNA3221::setup(Error &err) {
        uint16_t mode = (config.enable_channel1 << 14) | (config.enable_channel1 << 13) | (config.enable_channel1 << 12) |
                ((uint16_t) config.averagingMode << 10) | ((uint16_t) config.bus_voltage_time << 6) |
                ((uint16_t) config.shunt_voltage_time << 3) | ((uint16_t) config.operating_mode);

        i2c_write(Register::CONFG, mode, err);

        if (err != Error::NO_ERRORS) {return;}

        auto[critical_threshold_1, warning_threshold_1] = config.threshold1;

        i2c_write(Register::CH1CA, voltage_conversion(critical_threshold_1, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::CH1WA, voltage_conversion(warning_threshold_1, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        auto[critical_threshold_2, warning_threshold_2] = config.threshold2;

        i2c_write(Register::CH2CA, voltage_conversion(critical_threshold_2, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::CH2WA, voltage_conversion(warning_threshold_2, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        auto[critical_threshold_3, warning_threshold_3] = config.threshold3;

        i2c_write(Register::CH3CA, voltage_conversion(critical_threshold_3, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::CH3WA, voltage_conversion(warning_threshold_3, 40, 3), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::SHVLL, voltage_conversion(config.shunt_voltage_sum_limit, 40, 1), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::PWRVU, voltage_conversion(config.power_valid_upper, 8000, 1), err);
        if (err != Error::NO_ERRORS) {return;}

        i2c_write(Register::PWRVL, voltage_conversion(config.power_valid_lower, 8000, 1), err);
        if (err != Error::NO_ERRORS) {return;}
    }

}
