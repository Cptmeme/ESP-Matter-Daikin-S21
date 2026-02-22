/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <log_heap_numbers.h>

#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif
using namespace chip::DeviceLayer;
#endif

// --- REQUIRED FOR AAI ---
#include <app/AttributeAccessInterface.h>
#include <app/AttributeAccessInterfaceRegistry.h>
#include <app/util/attribute-storage.h>

static const char *TAG = "app_main";
uint16_t thermostat_endpoint_id = 0;
uint16_t powerful_endpoint_id = 0;

// Extern the global variable from app_driver.cpp
extern int16_t g_current_temp_int; 

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

// --- ATTRIBUTE ACCESSOR CLASS ---
class LocalTempAccessor : public chip::app::AttributeAccessInterface
{
public:
    LocalTempAccessor() : AttributeAccessInterface(chip::Optional<chip::EndpointId>::Missing(), Thermostat::Id) {}

    CHIP_ERROR Read(const chip::app::ConcreteReadAttributePath & aPath, chip::app::AttributeValueEncoder & aEncoder) override
    {
        if (aPath.mAttributeId == Thermostat::Attributes::LocalTemperature::Id)
        {
            return aEncoder.Encode(g_current_temp_int);
        }
        return CHIP_NO_ERROR;
    }
};

static LocalTempAccessor sLocalTempAccessor;
// --------------------------------

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        MEMORY_PROFILER_DUMP_HEAP_STAT("commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        MEMORY_PROFILER_DUMP_HEAP_STAT("commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        MEMORY_PROFILER_DUMP_HEAP_STAT("BLE deinitialized");
        break;

    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE) {
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }
    return err;
}

// Helper to safely create attributes
static void ensure_attribute(esp_matter::cluster_t *cluster, uint32_t attribute_id, uint8_t type, esp_matter_attr_val_t val) {
    esp_matter::attribute_t *attr = esp_matter::attribute::get(cluster, attribute_id);
    if (!attr) {
        esp_matter::attribute::create(cluster, attribute_id, ATTRIBUTE_FLAG_NULLABLE, val);
    }
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;
    nvs_flash_init();

    app_driver_handle_t thermostat_handle = app_driver_thermostat_init();
    
    // --- ENABLE BUTTON ---
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle); // This handles Factory Reset (Long Press)
    // ---------------------

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    esp_matter::endpoint::thermostat::config_t thermostat_config = {};
    thermostat_config.thermostat.feature_flags = 35; // Heat/Cool/Auto
    //thermostat_config.thermostat.system_mode = 0;
    thermostat_config.thermostat.control_sequence_of_operation = 4;

    endpoint_t *endpoint = esp_matter::endpoint::thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, thermostat_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create thermostat endpoint"));

    thermostat_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", thermostat_endpoint_id);

    esp_matter::endpoint::on_off_plug_in_unit::config_t powerful_config;
    powerful_config.on_off.on_off = false; // Default state

    // Create the plug_in unit, passing the thermostat_handle so it routes to the same app_driver_attribute_update callback
    endpoint_t *powerful_ep = esp_matter::endpoint::on_off_plug_in_unit::create(node, &powerful_config, ENDPOINT_FLAG_NONE, thermostat_handle);
    ABORT_APP_ON_FAILURE(powerful_ep != nullptr, ESP_LOGE(TAG, "Failed to create powerful endpoint"));

    powerful_endpoint_id = endpoint::get_id(powerful_ep);
    ESP_LOGI(TAG, "Powerful plug_in unit created with endpoint_id %d", powerful_endpoint_id);

    // --- MANUALLY REGISTER ATTRIBUTES ---
    esp_matter::cluster_t *cluster = esp_matter::cluster::get(endpoint, Thermostat::Id);
    if (cluster) {
        // 1. Local Temperature (Sensor)
        ensure_attribute(cluster, Thermostat::Attributes::LocalTemperature::Id, ESP_MATTER_VAL_TYPE_INT16, esp_matter_int16(2100));
        
        // 2. System Mode (Control)
        ensure_attribute(cluster, Thermostat::Attributes::SystemMode::Id, ESP_MATTER_VAL_TYPE_ENUM8, esp_matter_enum8(0));
        
        // 3. Cooling Setpoint
        ensure_attribute(cluster, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, ESP_MATTER_VAL_TYPE_INT16, esp_matter_int16(2000));

        // 4. Heating Setpoint
        ensure_attribute(cluster, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, ESP_MATTER_VAL_TYPE_INT16, esp_matter_int16(2000));

        // 5. Running State (Fixes the "Off when Idle" bug)
        // 0=Idle, 1=Heat, 2=Cool
        ensure_attribute(cluster, Thermostat::Attributes::ThermostatRunningState::Id, ESP_MATTER_VAL_TYPE_BITMAP16, esp_matter_bitmap16(0));
    }
    // ------------------------------------

    // --- REGISTER THE ACCESSOR ---
    chip::app::AttributeAccessInterfaceRegistry::Instance().Register(&sLocalTempAccessor);
    // -----------------------------

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    // Enable secondary network interface
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint = endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create secondary network interface endpoint"));
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
    auto * dac_provider = get_dac_provider();
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
    static_cast<ESP32SecureCertDACProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
    static_cast<ESP32FactoryDataProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#endif
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    app_driver_thermostat_set_defaults(thermostat_endpoint_id);
    
#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::attribute_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

    while (true) {
        MEMORY_PROFILER_DUMP_HEAP_STAT("Idle");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}