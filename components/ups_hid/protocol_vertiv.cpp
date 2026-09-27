#include "protocol_vertiv.h"

#include "constants_hid.h"
#include "constants_ups.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace ups_hid {

static const char *const VERTIV_TAG = "ups_hid.vertiv";

// -----------------------------------------------------------------------------
// Detection / initialization
// -----------------------------------------------------------------------------

bool VertivHidProtocol::detect()
{
    if (!parent_->is_connected())
    {
        return false;
    }

    const uint16_t vendor_id = parent_->get_vendor_id();
    const uint16_t product_id = parent_->get_product_id();

    if (vendor_id != VERTIV_VENDOR_ID)
    {
        return false;
    }

    // Be deliberately conservative. 0x10AF has been used across several
    // Liebert/Vertiv products, but the descriptor layout implemented below
    // is specifically the 0x0002 family.
    if (product_id != VERTIV_PRODUCT_ID_0002)
    {
        ESP_LOGD(
            VERTIV_TAG,
            "Vertiv/Liebert VID matched but PID 0x%04X is not supported by this "
            "protocol",
            product_id);

        return false;
    }

    HidReport status_report;

    if (!read_input_or_feature_report(REPORT_POWER_SUMMARY_STATUS, status_report))
    {
        ESP_LOGW(
            VERTIV_TAG,
            "10AF:0002 detected, but PowerSummary PresentStatus report 0x32 "
            "could not be read");

        return false;
    }

    // Report 0x32 needs at least 24 bits of payload for the status fields
    // through NeedReplacement at bit 23.
    const size_t payload = payload_offset(status_report);

    if (status_report.bytes.size() < payload + 3)
    {
        ESP_LOGW(
            VERTIV_TAG,
            "Report 0x32 too short: %zu bytes (payload begins at %zu)",
            status_report.bytes.size(),
            payload);

        return false;
    }

    ESP_LOGI(
        VERTIV_TAG,
        "Detected Vertiv/Liebert HID device VID=0x%04X PID=0x%04X",
        vendor_id,
        product_id);

    return true;
}

bool VertivHidProtocol::initialize()
{
    if (!parent_->is_connected())
    {
        return false;
    }

    ESP_LOGI(VERTIV_TAG, "Initializing Vertiv/Liebert HID protocol");

    cache_static_data();

    return true;
}

// -----------------------------------------------------------------------------
// Main poll
// -----------------------------------------------------------------------------

bool VertivHidProtocol::read_data(UpsData &data)
{
    if (!parent_->is_connected())
    {
        return false;
    }

    populate_static_data(data);

    bool read_any_measurement = false;

    // ---------------------------------------------------------------------------
    // Battery charge
    // ---------------------------------------------------------------------------

    uint8_t u8_value = 0;

    if (read_dynamic_u8(REPORT_REMAINING_CAPACITY, u8_value) && u8_value <= 100)
    {
        data.battery.level = static_cast<float>(u8_value);
        read_any_measurement = true;

        ESP_LOGD(
            VERTIV_TAG,
            "Battery charge: %.0f%%",
            data.battery.level);
    }
    else if (read_feature_u8(REPORT_BATTERY_REMAINING_CAPACITY, u8_value) && u8_value <= 100)
    {
        data.battery.level = static_cast<float>(u8_value);
        read_any_measurement = true;

        ESP_LOGD(
            VERTIV_TAG,
            "Battery charge (BatterySystem fallback): %.0f%%",
            data.battery.level);
    }

    // ---------------------------------------------------------------------------
    // Runtime to empty
    // HID descriptor defines this report as a 16-bit value in seconds.
    // ---------------------------------------------------------------------------

    uint16_t u16_value = 0;

    if (read_dynamic_u16(REPORT_RUNTIME_TO_EMPTY, u16_value) && u16_value != 0xFFFF)
    {
        data.battery.runtime_minutes = static_cast<float>(u16_value) / 60.0f;

        read_any_measurement = true;

        ESP_LOGD(
            VERTIV_TAG,
            "Battery runtime: %u seconds (%.1f minutes)",
            u16_value,
            data.battery.runtime_minutes);
    }

    // ---------------------------------------------------------------------------
    // Battery voltage
    //
    // This Vertiv/Liebert family is known to advertise broken HID unit
    // exponents. The raw integer itself is useful: for example raw 524
    // corresponds to 52.4 V.
    // ---------------------------------------------------------------------------

    if (read_feature_u16(REPORT_BATTERY_VOLTAGE, u16_value))
    {
        const float voltage = decode_scaled_value(u16_value, 5.0f, 200.0f);

        if (!std::isnan(voltage))
        {
            data.battery.voltage = voltage;
            read_any_measurement = true;

            ESP_LOGD(
                VERTIV_TAG,
                "Battery voltage: %.1f V (raw=%u)",
                voltage,
                u16_value);
        }
    }

    // ---------------------------------------------------------------------------
    // Input voltage
    // ---------------------------------------------------------------------------

    float measured_input_voltage = NAN;

    if (read_feature_u16(REPORT_INPUT_VOLTAGE, u16_value))
    {
        measured_input_voltage = decode_scaled_value(u16_value, 50.0f, 300.0f);

        if (!std::isnan(measured_input_voltage))
        {
            read_any_measurement = true;

            ESP_LOGD(
                VERTIV_TAG,
                "Input voltage: %.1f V (raw=%u)",
                measured_input_voltage,
                u16_value);
        }
    }

    // ---------------------------------------------------------------------------
    // Output voltage
    // ---------------------------------------------------------------------------

    if (read_feature_u16(REPORT_OUTPUT_VOLTAGE, u16_value))
    {
        const float voltage = decode_scaled_value(u16_value, 50.0f, 300.0f);

        if (!std::isnan(voltage))
        {
            data.power.output_voltage = voltage;
            read_any_measurement = true;

            ESP_LOGD(
                VERTIV_TAG,
                "Output voltage: %.1f V (raw=%u)",
                voltage,
                u16_value);
        }
    }

    // ---------------------------------------------------------------------------
    // Frequency
    //
    // The common 10AF:0002 descriptor gives raw values such as 500 or 600
    // for 50.0 / 60.0 Hz. decode_scaled_value() also tolerates firmware
    // variants that return the value directly or with another decade.
    // ---------------------------------------------------------------------------

    bool frequency_set = false;

    if (read_feature_u16(REPORT_INPUT_FREQUENCY, u16_value))
    {
        const float frequency = decode_scaled_value(u16_value, 40.0f, 70.0f);

        if (!std::isnan(frequency))
        {
            data.power.frequency = frequency;
            frequency_set = true;
            read_any_measurement = true;

            ESP_LOGD(
                VERTIV_TAG,
                "Input frequency: %.1f Hz (raw=%u)",
                frequency,
                u16_value);
        }
    }

    // The current UpsData model has one frequency field. Fall back to output
    // frequency only if input frequency was unavailable.
    if (!frequency_set && read_feature_u16(REPORT_OUTPUT_FREQUENCY, u16_value))
    {
        const float frequency = decode_scaled_value(u16_value, 40.0f, 70.0f);

        if (!std::isnan(frequency))
        {
            data.power.frequency = frequency;
            read_any_measurement = true;

            ESP_LOGD(
                VERTIV_TAG,
                "Output frequency used as frequency fallback: %.1f Hz (raw=%u)",
                frequency,
                u16_value);
        }
    }

    // ---------------------------------------------------------------------------
    // Load percentage
    // Some 10AF:0002 firmware exposes this report but rejects GET_REPORT for it.
    // Failure is therefore intentionally non-fatal.
    // ---------------------------------------------------------------------------

    if (read_feature_u8(REPORT_LOAD_PERCENT, u8_value) && u8_value <= 100)
    {
        data.power.load_percent = static_cast<float>(u8_value);
        read_any_measurement = true;

        ESP_LOGD(
            VERTIV_TAG,
            "UPS load: %.0f%%",
            data.power.load_percent);
    }

    // ---------------------------------------------------------------------------
    // Authoritative power/battery state.
    //
    // DO NOT derive OL/OB from arbitrary generic bits. Report 0x32 bit 16 is
    // explicitly ACPresent in the device's HID Power Device descriptor.
    // ---------------------------------------------------------------------------

    StatusFlags flags;

    if (!read_status(flags))
    {
        ESP_LOGW(
            VERTIV_TAG,
            "Unable to read authoritative Vertiv status report 0x32");

        // Do not manufacture an Online/On-Battery state from voltage alone.
        // Leaving input_voltage invalid causes callers to treat this read as
        // unusable rather than trusting an invented state.
        data.power.input_voltage = NAN;

        return false;
    }

    if (flags.ac_present)
    {
        data.power.status = status::ONLINE;

        // UpsHidComponent::is_online() currently derives OL/OB from whether
        // input_voltage is valid, so populate it whenever ACPresent is true.
        if (!std::isnan(measured_input_voltage))
        {
            data.power.input_voltage = measured_input_voltage;
        }
        else
        {
            data.power.input_voltage = parent_->get_fallback_nominal_voltage();
        }
    }
    else
    {
        data.power.status = status::ON_BATTERY;

        // This is intentional. The component's current NUT status layer uses
        // input_voltage validity as the OL/OB discriminator.
        data.power.input_voltage = NAN;
    }

    // ---------------------------------------------------------------------------
    // Human-readable battery status
    // ---------------------------------------------------------------------------

    if (flags.fully_discharged)
    {
        data.battery.status = "Fully Discharged";
    }
    else if (flags.discharging)
    {
        data.battery.status = "Discharging";
    }
    else if (flags.fully_charged)
    {
        data.battery.status = "Fully Charged";
    }
    else if (flags.charging)
    {
        data.battery.status = "Charging";
    }
    else
    {
        data.battery.status = "Normal";
    }

    if (flags.below_remaining_capacity)
    {
        append_status(data.battery.status, "Low");
    }

    if (flags.need_replacement)
    {
        append_status(data.battery.status, "Replace Battery");
    }

    if (flags.internal_failure)
    {
        append_status(data.battery.status, "Internal Failure");
    }

    if (flags.shutdown_imminent)
    {
        append_status(data.battery.status, "Shutdown Imminent");
    }

    // BatteryData::is_low() currently relies on charge_low rather than a
    // dedicated HID status flag. Preserve the configured warning threshold,
    // but when the UPS explicitly asserts BelowRemainingCapacityLimit make
    // sure the current charge satisfies the component's existing low-battery
    // predicate.
    if (flags.below_remaining_capacity && !std::isnan(data.battery.level))
    {
        if (std::isnan(data.battery.charge_low)
            || data.battery.level > data.battery.charge_low)
        {
            data.battery.charge_low = data.battery.level;
        }
    }

    ESP_LOGD(
        VERTIV_TAG,
        "Status: AC=%s charging=%s discharging=%s full=%s low=%s "
        "replace=%s internal_failure=%s shutdown_imminent=%s",
        flags.ac_present ? "YES" : "NO",
        flags.charging ? "YES" : "NO",
        flags.discharging ? "YES" : "NO",
        flags.fully_charged ? "YES" : "NO",
        flags.below_remaining_capacity ? "YES" : "NO",
        flags.need_replacement ? "YES" : "NO",
        flags.internal_failure ? "YES" : "NO",
        flags.shutdown_imminent ? "YES" : "NO");

    return read_any_measurement || true;
}

// -----------------------------------------------------------------------------
// Status report 0x32
// -----------------------------------------------------------------------------

bool VertivHidProtocol::read_status(StatusFlags &flags) const
{
    HidReport report;

    if (!read_input_or_feature_report(REPORT_POWER_SUMMARY_STATUS, report))
    {
        return false;
    }

    // These are bit offsets within the report payload (the report-ID byte is
    // excluded from the offsets).
    //
    //  7  PowerSummary.PresentStatus.InternalFailure
    //  9  PowerSummary.PresentStatus.ShutdownImminent
    // 16  PowerSummary.PresentStatus.ACPresent
    // 18  PowerSummary.PresentStatus.BelowRemainingCapacityLimit
    // 19  PowerSummary.PresentStatus.FullyCharged
    // 20  PowerSummary.PresentStatus.Charging
    // 21  PowerSummary.PresentStatus.Discharging
    // 22  PowerSummary.PresentStatus.FullyDischarged
    // 23  PowerSummary.PresentStatus.NeedReplacement

    if (!payload_bit(report, 7, flags.internal_failure) ||
        !payload_bit(report, 9, flags.shutdown_imminent) ||
        !payload_bit(report, 16, flags.ac_present) ||
        !payload_bit(report, 18, flags.below_remaining_capacity) ||
        !payload_bit(report, 19, flags.fully_charged) ||
        !payload_bit(report, 20, flags.charging) ||
        !payload_bit(report, 21, flags.discharging) ||
        !payload_bit(report, 22, flags.fully_discharged) ||
        !payload_bit(report, 23, flags.need_replacement))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Static data
// -----------------------------------------------------------------------------

void VertivHidProtocol::cache_static_data()
{
    manufacturer_.clear();
    model_.clear();
    serial_.clear();
    battery_type_.clear();

    read_string_index(REPORT_MANUFACTURER_STRING, manufacturer_);
    read_string_index(REPORT_PRODUCT_STRING, model_);
    read_string_index(REPORT_SERIAL_STRING, serial_);
    read_string_index(REPORT_CHEMISTRY_STRING, battery_type_);

    // Descriptor/string access differs slightly between firmware revisions.
    // These fallbacks still produce useful NUT identity if a string feature
    // report is absent.
    if (manufacturer_.empty())
    {
        manufacturer_ = "Vertiv Co.";
    }

    if (model_.empty())
    {
        model_ = "Vertiv/Liebert USB UPS";
    }

    uint8_t u8_value = 0;
    uint16_t u16_value = 0;

    // 0x30 is the PowerSummary.Input.ConfigVoltage report found in the
    // 10AF:0002 descriptor. Fall back to UPS.Flow.ConfigVoltage (0x01).
    if (read_feature_u8(REPORT_INPUT_CONFIG_VOLTAGE, u8_value) && u8_value >= 50)
    {
        nominal_input_voltage_ = static_cast<float>(u8_value);
    }
    else if (read_feature_u8(REPORT_FLOW_CONFIG_VOLTAGE, u8_value) && u8_value >= 50)
    {
        nominal_input_voltage_ = static_cast<float>(u8_value);
    }

    if (read_feature_u16(REPORT_BATTERY_CONFIG_VOLTAGE, u16_value) && u16_value > 0)
    {
        nominal_battery_voltage_ = static_cast<float>(u16_value);
    }

    if (read_feature_u16(REPORT_CONFIG_APPARENT_POWER, u16_value) && u16_value > 0)
    {
        apparent_power_nominal_ = static_cast<float>(u16_value);
    }

    if (read_feature_u8(REPORT_WARNING_CAPACITY, u8_value) && u8_value <= 100)
    {
        warning_capacity_ = static_cast<float>(u8_value);
    }

    ESP_LOGI(
        VERTIV_TAG,
        "Vertiv static data: manufacturer='%s', model='%s', serial='%s', "
        "nominal_input=%.1fV, nominal_battery=%.1fV, nominal_power=%.0fVA",
        manufacturer_.c_str(),
        model_.c_str(),
        serial_.c_str(),
        nominal_input_voltage_,
        nominal_battery_voltage_,
        apparent_power_nominal_);
}

void VertivHidProtocol::populate_static_data(UpsData &data) const
{
    data.device.manufacturer = manufacturer_;
    data.device.model = model_;
    data.device.serial_number = serial_;

    data.device.usb_vendor_id = VERTIV_VENDOR_ID;
    data.device.usb_product_id = parent_->get_product_id();

    data.device.capabilities.supports_hid_get_report = true;
    data.device.capabilities.supports_runtime_estimation = true;
    data.device.capabilities.supports_configuration_queries = true;

    if (!battery_type_.empty())
    {
        data.battery.type = battery_type_;
    }

    if (!std::isnan(nominal_input_voltage_))
    {
        data.power.input_voltage_nominal = nominal_input_voltage_;
        data.power.output_voltage_nominal = nominal_input_voltage_;
    }

    if (!std::isnan(nominal_battery_voltage_))
    {
        data.battery.voltage_nominal = nominal_battery_voltage_;
    }

    if (!std::isnan(apparent_power_nominal_))
    {
        data.power.apparent_power_nominal = apparent_power_nominal_;
    }

    if (!std::isnan(warning_capacity_))
    {
        data.battery.charge_warning = warning_capacity_;

        // The current component exposes only a numeric low threshold to
        // BatteryData::is_low(). This is the closest descriptor-backed threshold
        // available on this family.
        data.battery.charge_low = warning_capacity_;
    }
}

// -----------------------------------------------------------------------------
// HID report helpers
// -----------------------------------------------------------------------------

bool VertivHidProtocol::read_report(uint8_t report_type, uint8_t report_id, HidReport &report) const
{
    uint8_t buffer[64] = {0};
    size_t length = sizeof(buffer);

    const esp_err_t result =
        parent_->hid_get_report(
            report_type,
            report_id,
            buffer,
            &length,
            parent_->get_protocol_timeout());

    if (result != ESP_OK || length == 0)
        return false;

    report.id = report_id;
    report.type = report_type;
    report.bytes.assign(buffer, buffer + length);

    ESP_LOGV(
        VERTIV_TAG,
        "Read HID report type=0x%02X id=0x%02X length=%zu",
        report_type,
        report_id,
        length);

    return true;
}

bool VertivHidProtocol::read_input_or_feature_report(uint8_t report_id, HidReport &report) const
{
    // Dynamic values on this family are often advertised both as Input and
    // Feature reports. Prefer Input, then fall back to Feature.
    if (read_report(HID_REPORT_TYPE_INPUT, report_id, report))
        return true;

    return read_report(HID_REPORT_TYPE_FEATURE, report_id, report);
}

size_t VertivHidProtocol::payload_offset(const HidReport &report) const
{
    if (!report.bytes.empty() &&
        report.bytes[0] == report.id) {
        return 1;
    }

    return 0;
}

bool VertivHidProtocol::payload_u8(const HidReport &report, size_t byte_offset, uint8_t &value) const
{
    const size_t index = payload_offset(report) + byte_offset;

    if (index >= report.bytes.size())
        return false;

    value = report.bytes[index];
    return true;
}

bool VertivHidProtocol::payload_u16_le(const HidReport &report, size_t byte_offset, uint16_t &value) const
{
    const size_t index = payload_offset(report) + byte_offset;

    if (index + 1 >= report.bytes.size())
        return false;

    value =
        static_cast<uint16_t>(report.bytes[index]) |
        (static_cast<uint16_t>(report.bytes[index + 1]) << 8);

    return true;
}

bool VertivHidProtocol::payload_bit(const HidReport &report, size_t bit_offset, bool &value) const
{
    const size_t byte_offset = bit_offset / 8;
    const uint8_t mask = static_cast<uint8_t>(1U << (bit_offset % 8));
    uint8_t byte = 0;

    if (!payload_u8(report, byte_offset, byte))
        return false;

    value = (byte & mask) != 0;
    return true;
}

bool VertivHidProtocol::read_feature_u8(uint8_t report_id, uint8_t &value) const
{
    HidReport report;
    if (!read_report(HID_REPORT_TYPE_FEATURE, report_id, report))
        return false;
    return payload_u8(report, 0, value);
}

bool VertivHidProtocol::read_feature_u16(uint8_t report_id, uint16_t &value) const
{
    HidReport report;
    if (!read_report(HID_REPORT_TYPE_FEATURE, report_id, report))
        return false;
  return payload_u16_le(report, 0, value);
}

bool VertivHidProtocol::read_dynamic_u8(uint8_t report_id, uint8_t &value) const
{
    HidReport report;
    if (!read_input_or_feature_report(report_id, report))
        return false;
    return payload_u8(report, 0, value);
}

bool VertivHidProtocol::read_dynamic_u16(uint8_t report_id, uint16_t &value) const
{
    HidReport report;
    if (!read_input_or_feature_report(report_id, report))
        return false;
    return payload_u16_le(report, 0, value);
}

bool VertivHidProtocol::read_string_index(uint8_t report_id, std::string &value) const
{
    uint8_t string_index = 0;

    if (!read_feature_u8(report_id, string_index))
        return false;

    if (string_index == 0)
        return false;

    std::string result;

    if (parent_->get_string_descriptor(string_index, result) != ESP_OK)
        return false;

    if (result.empty())
        return false;

    value = result;
    return true;
}

// -----------------------------------------------------------------------------
// Scaling helpers
// -----------------------------------------------------------------------------

float VertivHidProtocol::decode_scaled_value(
    uint16_t raw,
    float minimum,
    float maximum)
{
    if (raw == 0 || raw == 0xFFFF)
        return NAN;

    // Most 10AF:0002 voltage/frequency reports are raw tenths.
    const float divided_by_10 = static_cast<float>(raw) / 10.0f;

    if (divided_by_10 >= minimum && divided_by_10 <= maximum)
        return divided_by_10;

    // Be tolerant of another known decade of broken HID exponent handling.
    const float divided_by_100 = static_cast<float>(raw) / 100.0f;

    if (divided_by_100 >= minimum && divided_by_100 <= maximum)
        return divided_by_100;

    // Some firmware may simply return the engineering-unit integer directly.
    const float direct = static_cast<float>(raw);

    if (direct >= minimum && direct <= maximum)
        return direct;

    return NAN;
}

void VertivHidProtocol::append_status(std::string &base, const char *suffix)
{
    if (suffix == nullptr || suffix[0] == '\0')
        return;

    if (base.empty())
    {
        base = suffix;
        return;
    }

    base += " - ";
    base += suffix;
}

}  // namespace ups_hid
}  // namespace esphome

// -----------------------------------------------------------------------------
// Protocol factory registration
//
// Keep this in addition to the explicit factory registration described below.
// The explicit reference from protocol_factory.cpp is required by this
// repository's static-library linking model.
// -----------------------------------------------------------------------------

#include "protocol_factory.h"

namespace esphome {
namespace ups_hid {

std::unique_ptr<UpsProtocolBase> create_vertiv_protocol(UpsHidComponent *parent)
{
    return std::make_unique<VertivHidProtocol>(parent);
}

}  // namespace ups_hid
}  // namespace esphome

REGISTER_UPS_PROTOCOL_FOR_VENDOR(
    0x10AF,
    vertiv_hid_protocol,
    esphome::ups_hid::create_vertiv_protocol,
    "Vertiv/Liebert HID Protocol",
    "Vertiv/Liebert 10AF:0002 HID Power Device protocol",
    100);