#pragma once

#include "ups_hid.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace ups_hid {

/**
 * Vertiv/Liebert USB HID protocol for the 10AF:0002 device family.
 *
 * This family includes Vertiv/Liebert PST5, PSI5, Edge and related UPSes
 * which expose the useful USB HID Power Device descriptor on interface 1.
 *
 * The report IDs and bit offsets used here are descriptor-derived rather
 * than the heuristic report interpretation used by GenericHidProtocol.
 */
class VertivHidProtocol final : public UpsProtocolBase
{
public:
    explicit VertivHidProtocol(UpsHidComponent *parent)
      : UpsProtocolBase(parent) {}

    ~VertivHidProtocol() override = default;

    bool detect() override;
    bool initialize() override;
    bool read_data(UpsData &data) override;

    DeviceInfo::DetectedProtocol get_protocol_type() const override
    {
        return DeviceInfo::PROTOCOL_VERTIV_HID;
    }

    std::string get_protocol_name() const override {
        return "Vertiv/Liebert HID";
    }

private:
    static constexpr uint16_t VERTIV_VENDOR_ID = 0x10AF;
    static constexpr uint16_t VERTIV_PRODUCT_ID_0002 = 0x0002;

    // -------------------------------------------------------------------------
    // HID report IDs derived from the 10AF:0002 Power Device descriptor.
    // -------------------------------------------------------------------------

    // UPS.Flow.ConfigVoltage
    static constexpr uint8_t REPORT_FLOW_CONFIG_VOLTAGE = 0x01;

    // UPS.Flow.ConfigApparentPower
    static constexpr uint8_t REPORT_CONFIG_APPARENT_POWER = 0x03;

    // UPS.BatterySystem.Battery.ConfigVoltage
    static constexpr uint8_t REPORT_BATTERY_CONFIG_VOLTAGE = 0x04;

    // UPS.PowerSummary.AudibleAlarmControl
    static constexpr uint8_t REPORT_AUDIBLE_ALARM = 0x11;

    // UPS.PowerConverter.Input.Voltage
    static constexpr uint8_t REPORT_INPUT_VOLTAGE = 0x18;

    // UPS.PowerConverter.Input.Frequency
    static constexpr uint8_t REPORT_INPUT_FREQUENCY = 0x19;

    // UPS.PowerConverter.Output.Voltage
    static constexpr uint8_t REPORT_OUTPUT_VOLTAGE = 0x1B;

    // UPS.PowerConverter.Output.Frequency
    static constexpr uint8_t REPORT_OUTPUT_FREQUENCY = 0x1C;

    // UPS.OutletSystem.Outlet.PercentLoad
    static constexpr uint8_t REPORT_LOAD_PERCENT = 0x1E;

    // UPS.BatterySystem.Battery.Voltage
    static constexpr uint8_t REPORT_BATTERY_VOLTAGE = 0x20;

    // Alternate battery remaining-capacity field
    static constexpr uint8_t REPORT_BATTERY_REMAINING_CAPACITY = 0x21;

    // UPS.BatterySystem.Battery.PresentStatus
    static constexpr uint8_t REPORT_BATTERY_PRESENT_STATUS = 0x23;

    // String-index reports
    static constexpr uint8_t REPORT_PRODUCT_STRING = 0x28;
    static constexpr uint8_t REPORT_SERIAL_STRING = 0x29;
    static constexpr uint8_t REPORT_CHEMISTRY_STRING = 0x2A;
    static constexpr uint8_t REPORT_MANUFACTURER_STRING = 0x2B;

    // UPS.PowerSummary.Input.ConfigVoltage
    static constexpr uint8_t REPORT_INPUT_CONFIG_VOLTAGE = 0x30;

    // UPS.PowerSummary.PresentStatus
    static constexpr uint8_t REPORT_POWER_SUMMARY_STATUS = 0x32;

    // UPS.PowerSummary.RemainingCapacity
    static constexpr uint8_t REPORT_REMAINING_CAPACITY = 0x34;

    // UPS.PowerSummary.RunTimeToEmpty
    static constexpr uint8_t REPORT_RUNTIME_TO_EMPTY = 0x35;

    // UPS.PowerSummary.WarningCapacityLimit
    static constexpr uint8_t REPORT_WARNING_CAPACITY = 0x38;

    struct HidReport {
        uint8_t id{0};
        uint8_t type{0};
        std::vector<uint8_t> bytes;
    };

    struct StatusFlags {
        bool internal_failure{false};
        bool shutdown_imminent{false};

        bool ac_present{false};

        bool below_remaining_capacity{false};
        bool fully_charged{false};
        bool charging{false};
        bool discharging{false};
        bool fully_discharged{false};
        bool need_replacement{false};
    };

    // HID transport helpers
    bool read_report(uint8_t report_type,
                    uint8_t report_id,
                    HidReport &report) const;

    bool read_input_or_feature_report(uint8_t report_id,
                                        HidReport &report) const;

    // HID GET_REPORT responses normally include the report ID as byte zero.
    // These helpers deliberately tolerate devices/transports which strip it.
    size_t payload_offset(const HidReport &report) const;

    bool payload_u8(const HidReport &report,
                    size_t byte_offset,
                    uint8_t &value) const;

    bool payload_u16_le(const HidReport &report,
                        size_t byte_offset,
                        uint16_t &value) const;

    bool payload_bit(const HidReport &report,
                    size_t bit_offset,
                    bool &value) const;

    bool read_feature_u8(uint8_t report_id, uint8_t &value) const;
    bool read_feature_u16(uint8_t report_id, uint16_t &value) const;

    bool read_dynamic_u8(uint8_t report_id, uint8_t &value) const;
    bool read_dynamic_u16(uint8_t report_id, uint16_t &value) const;

    bool read_string_index(uint8_t report_id, std::string &value) const;

    // Actual 10AF:0002 status parser
    bool read_status(StatusFlags &flags) const;

    // Cache data that should not need to be re-read every 10 seconds
    void cache_static_data();
    void populate_static_data(UpsData &data) const;

    // Liebert/Vertiv firmware in this family is known to publish incorrect HID
    // unit exponents for several measurements. Decode the raw integer while
    // accepting the common x10/x100 representations conservatively.
    static float decode_scaled_value(uint16_t raw,
                                    float minimum,
                                    float maximum);

    static void append_status(std::string &base, const char *suffix);

    // Cached static values
    std::string manufacturer_;
    std::string model_;
    std::string serial_;
    std::string battery_type_;

    float nominal_input_voltage_{NAN};
    float nominal_battery_voltage_{NAN};
    float apparent_power_nominal_{NAN};
    float warning_capacity_{NAN};
};

std::unique_ptr<UpsProtocolBase>
create_vertiv_protocol(UpsHidComponent *parent);

}  // namespace ups_hid
}  // namespace esphome