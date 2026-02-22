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
extern uint16_t thermostat_endpoint_id;
static DaikinS21 s21;

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

    // --- 1. Update Local Temp ---
    int16_t new_temp = FLOAT_TO_MATTER(data->state.current_temp);
    if (g_current_temp_int != new_temp) {
        g_current_temp_int = new_temp;
        MatterReportingAttributeChangeCallback(
            thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id);
    }

    // --- 2. Update Target Temp ---
    esp_matter_attr_val_t val = esp_matter_int16(FLOAT_TO_MATTER(data->state.target_temp));
    if (data->state.mode == FAIKIN_MODE_HEAT) {
        esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &val);
    } else {
        esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, &val);
    }

    // --- 3. Update System Mode ---
    uint8_t matter_mode = 0; // Off
    if (data->state.power) {
        switch(data->state.mode) {
            case FAIKIN_MODE_AUTO: matter_mode = 1; break; 
            case FAIKIN_MODE_COOL: matter_mode = 3; break; 
            case FAIKIN_MODE_HEAT: matter_mode = 4; break; 
            default: matter_mode = 1; break; 
        }
    }
    val = esp_matter_enum8(matter_mode);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &val);

    // --- 4. Update Running State (Idle vs Active) ---
    // 0=Idle, 1=Heat, 2=Cool (Bitmap)
    uint16_t running_state = 0; 
    
    if (data->state.power) {
        if (data->state.mode == FAIKIN_MODE_HEAT && data->state.current_temp < data->state.target_temp) {
             running_state = 1; // Active Heating
        } else if (data->state.mode == FAIKIN_MODE_COOL && data->state.current_temp > data->state.target_temp) {
             running_state = 2; // Active Cooling
        }
        // Else (at temp) -> 0 (Idle)
    }
    
    val = esp_matter_bitmap16(running_state);
    esp_matter::attribute::report(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &val);
    // ------------------------------------------------

    free(data);
}

static void s21_state_change_callback(const ac_state_t *state)
{
    if (thermostat_endpoint_id == 0) return;
    AppEventData *data = (AppEventData *)malloc(sizeof(AppEventData));
    if (data) {
        data->state = *state;
        chip::DeviceLayer::PlatformMgr().ScheduleWork(AppDriverUpdateTask, (intptr_t)data);
    }
}

static esp_err_t app_driver_thermostat_set_value(void *handle, esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    // Get current state to check power status
    ac_state_t current = s21.GetState();

    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
        uint8_t mode = val->val.u8;
        
        if (mode == 0) {
            // CASE: OFF
            // Explicitly set power off. 
            s21.SetPower(false);
        } else {
            // CASE: AUTO/COOL/HEAT
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

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id) {
        return app_driver_thermostat_set_value(driver_handle, val, attribute_id);
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