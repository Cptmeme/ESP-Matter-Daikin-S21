/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app_priv.h>
#include <common_macros.h>
#include <app/reporting/reporting.h> 
#include <platform/CHIPDeviceLayer.h>

#include "iot_button.h"
#include "button_gpio.h"
#include <app/server/Server.h>
#include <app/server/CommissioningWindowManager.h>

#include "s21_driver.h"

using namespace chip::app::Clusters;
using namespace chip::app::Clusters::Thermostat;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t ac_endpoint_id;
extern uint16_t powerful_endpoint_id;
static DaikinS21 s21;

static uint8_t s21_fan_to_matter(uint8_t s21_fan)
{
    switch (s21_fan) {
        case FAIKIN_FAN_AUTO:  return 5; // Auto
        case FAIKIN_FAN_QUIET:
        case FAIKIN_FAN_1:
        case FAIKIN_FAN_2:     return 1; // Low
        case FAIKIN_FAN_3:     return 2; // Medium
        case FAIKIN_FAN_4:
        case FAIKIN_FAN_5:     return 3; // High
        default:               return 5;
    }
}

static uint8_t matter_fan_to_s21(uint8_t matter_fan)
{
    switch (matter_fan) {
        case 1: return FAIKIN_FAN_1; // Low
        case 2: return FAIKIN_FAN_3; // Medium
        case 3: return FAIKIN_FAN_5; // High
        default: return FAIKIN_FAN_AUTO; // Off / On / Auto / Smart
    }
}

static uint8_t s21_fan_to_percent(uint8_t s21_fan)
{
    switch (s21_fan) {
        case FAIKIN_FAN_QUIET:
        case FAIKIN_FAN_1: return 20;
        case FAIKIN_FAN_2: return 40;
        case FAIKIN_FAN_3: return 60;
        case FAIKIN_FAN_4: return 80;
        case FAIKIN_FAN_5: return 100;
        default:           return 0;
    }
}

static uint8_t percent_to_s21_fan(uint8_t percent)
{
    if (percent == 0)  return FAIKIN_FAN_AUTO;
    if (percent <= 20) return FAIKIN_FAN_1;
    if (percent <= 40) return FAIKIN_FAN_2;
    if (percent <= 60) return FAIKIN_FAN_3;
    if (percent <= 80) return FAIKIN_FAN_4;
    return FAIKIN_FAN_5;
}

// Global Temperature Storage
int16_t g_current_temp_int = 2100; 

#define S21_TX_PIN 21
#define S21_RX_PIN 20
#define BUTTON_GPIO_PIN 23

#define FLOAT_TO_MATTER(x) ((int16_t)((x) * 100.0f))
#define MATTER_TO_FLOAT(x) ((float)(x) / 100.0f)

static void s21_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "S21 Poll Task Started");
    while (1) {
        s21.Poll();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

struct AppEventData {
    ac_state_t state;
};

static void AppDriverUpdateTask(intptr_t context)
{
    AppEventData *data = (AppEventData *)context;
    if (!data) return;

    // Local temperature (Thermostat cluster, served via LocalTempAccessor).
    int16_t new_temp = FLOAT_TO_MATTER(data->state.current_temp);
    if (g_current_temp_int != new_temp) {
        g_current_temp_int = new_temp;
        MatterReportingAttributeChangeCallback(
            ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id);
    }

    // Mirror the same reading on the TemperatureMeasurement cluster (used by the AC tile).
    esp_matter_attr_val_t val = esp_matter_nullable_int16(new_temp);
    esp_matter::attribute::report(ac_endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);

    // Target temperature.
    val = esp_matter_int16(FLOAT_TO_MATTER(data->state.target_temp));
    if (data->state.mode == FAIKIN_MODE_HEAT) {
        esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &val);
    } else {
        esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, &val);
    }

    // OnOff cluster — explicit power state for the Room AC tile.
    val = esp_matter_bool(data->state.power);
    esp_matter::attribute::report(ac_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val);

    // Thermostat SystemMode — only meaningful while powered; report current mode otherwise.
    uint8_t matter_mode = 0; // Off
    if (data->state.power) {
        switch (data->state.mode) {
            case FAIKIN_MODE_AUTO: matter_mode = 1; break;
            case FAIKIN_MODE_COOL: matter_mode = 3; break;
            case FAIKIN_MODE_HEAT: matter_mode = 4; break;
            default: matter_mode = 1; break;
        }
    }
    val = esp_matter_enum8(matter_mode);
    esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &val);

    // Running state (0=Idle, bit0=Heat, bit1=Cool).
    uint16_t running_state = 0;
    if (data->state.power) {
        if (data->state.mode == FAIKIN_MODE_HEAT && data->state.current_temp < data->state.target_temp) {
            running_state = 1;
        } else if (data->state.mode == FAIKIN_MODE_COOL && data->state.current_temp > data->state.target_temp) {
            running_state = 2;
        }
    }
    val = esp_matter_bitmap16(running_state);
    esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &val);

    // Fan Control — mirror Daikin fan speed into Matter.
    val = esp_matter_enum8(s21_fan_to_matter(data->state.fan_speed));
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::FanMode::Id, &val);

    uint8_t pct = s21_fan_to_percent(data->state.fan_speed);
    val = esp_matter_nullable_uint8(pct);
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::PercentSetting::Id, &val);
    val = esp_matter_uint8(pct);
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::PercentCurrent::Id, &val);

    if (powerful_endpoint_id != 0) {
        val = esp_matter_bool(data->state.powerful);
        esp_matter::attribute::report(powerful_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val);
    }

    free(data);
}

static void s21_state_change_callback(const ac_state_t *state)
{
    if (ac_endpoint_id == 0) return;
    AppEventData *data = (AppEventData *)malloc(sizeof(AppEventData));
    if (data) {
        data->state = *state;
        chip::DeviceLayer::PlatformMgr().ScheduleWork(AppDriverUpdateTask, (intptr_t)data);
    }
}

static esp_err_t app_driver_thermostat_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    ac_state_t current = s21.GetState();

    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
        uint8_t mode = val->val.u8;
        if (mode == 0) {
            s21.SetPower(false);
        } else {
            if (!current.power) {
                s21.SetPower(true);
            }
            if (mode == 1) s21.SetMode(FAIKIN_MODE_AUTO);
            else if (mode == 3) s21.SetMode(FAIKIN_MODE_COOL);
            else if (mode == 4) s21.SetMode(FAIKIN_MODE_HEAT);
        }
    }
    else if (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id ||
             attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
        if (current.power) {
            s21.SetTemp(MATTER_TO_FLOAT(val->val.i16));
        } else {
            ESP_LOGI(TAG, "Ignored SetTemp because device is OFF");
        }
    }
    return ESP_OK;
}

static esp_err_t app_driver_on_off_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == OnOff::Attributes::OnOff::Id) {
        ESP_LOGI(TAG, "Setting Power to: %d", val->val.b);
        s21.SetPower(val->val.b);
    }
    return ESP_OK;
}

static esp_err_t app_driver_fan_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == FanControl::Attributes::FanMode::Id) {
        uint8_t s21_fan = matter_fan_to_s21(val->val.u8);
        ESP_LOGI(TAG, "FanMode write %u → s21 %u", val->val.u8, s21_fan);
        s21.SetFan(s21_fan);
    }
    else if (attribute_id == FanControl::Attributes::PercentSetting::Id) {
        // Nullable uint8: when null, controller wants Auto.
        uint8_t s21_fan = (val->type == ESP_MATTER_VAL_TYPE_NULLABLE_UINT8 && val->val.u8 == 0xFF)
                          ? FAIKIN_FAN_AUTO
                          : percent_to_s21_fan(val->val.u8);
        ESP_LOGI(TAG, "PercentSetting write %u → s21 %u", val->val.u8, s21_fan);
        s21.SetFan(s21_fan);
    }
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id == powerful_endpoint_id && cluster_id == OnOff::Id) {
        if (attribute_id == OnOff::Attributes::OnOff::Id) {
            ESP_LOGI(TAG, "Setting Powerful Mode to: %d", val->val.b);
            s21.SetPowerful(val->val.b);
        }
        return ESP_OK;
    }
    if (endpoint_id != ac_endpoint_id) {
        return ESP_OK;
    }
    if (cluster_id == Thermostat::Id) {
        return app_driver_thermostat_set_value(val, attribute_id);
    }
    if (cluster_id == OnOff::Id) {
        return app_driver_on_off_set_value(val, attribute_id);
    }
    if (cluster_id == FanControl::Id) {
        return app_driver_fan_set_value(val, attribute_id);
    }
    return ESP_OK;
}

esp_err_t app_driver_thermostat_set_defaults(uint16_t endpoint_id) { return ESP_OK; }

app_driver_handle_t app_driver_thermostat_init()
{
    s21.Init(S21_TX_PIN, S21_RX_PIN);
    s21.SetStateCallback(s21_state_change_callback);
    xTaskCreate(s21_poll_task, "s21_poll", 4096, NULL, 5, NULL);
    return (app_driver_handle_t)1;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button Pressed: Opening Commissioning Window");
    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (!commissionMgr.IsCommissioningWindowOpen()) {
        commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                   chip::CommissioningWindowAdvertisement::kDnssdOnly);
    }
}

app_driver_handle_t app_driver_button_init()
{
    button_config_t btn_cfg = {0};
    button_gpio_config_t btn_gpio_cfg = { .gpio_num = BUTTON_GPIO_PIN, .active_level = 0 };
    button_handle_t btn_handle = NULL;
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle);
    if (err == ESP_OK && btn_handle) {
        iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
        return (app_driver_handle_t)btn_handle;
    }
    return NULL;
}