// ElectricalEnergyMeasurement (0x0091) — exposes the AC's lifetime total energy
// (S21 FM query) as CumulativeEnergyImported. Kept in its own translation unit so
// its `using namespace ElectricalEnergyMeasurement` doesn't collide with the
// ElectricalPowerMeasurement usings in power_measurement.cpp. Pattern taken from
// esp-matter's all_device_types_app electrical_measurement example: an
// ElectricalEnergyMeasurementAttrAccess serves the (MANAGED_INTERNALLY) attributes,
// SetMeasurementAccuracy publishes the mandatory accuracy, and
// NotifyCumulativeEnergyMeasured updates the value.

#include "power_measurement.h"   // declares energy_measurement_init / _set_wh

#include <memory>
#include <esp_log.h>
#include <app/data-model/List.h>
#include <app/clusters/electrical-energy-measurement-server/ElectricalEnergyMeasurementCluster.h>
#include <app/clusters/electrical-energy-measurement-server/electrical-energy-measurement-server.h>
#include <system/SystemClock.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::DataModel;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::ElectricalEnergyMeasurement;
using namespace chip::app::Clusters::ElectricalEnergyMeasurement::Structs;

static const char *TAG = "energy_measurement";

static std::unique_ptr<ElectricalEnergyMeasurementAttrAccess> s_eem_attr_access;

esp_err_t energy_measurement_init(uint16_t endpoint_id)
{
    if (s_eem_attr_access) {
        return ESP_OK;
    }

    s_eem_attr_access = std::make_unique<ElectricalEnergyMeasurementAttrAccess>(
        BitMask<Feature, uint32_t>(Feature::kImportedEnergy, Feature::kCumulativeEnergy),
        BitMask<OptionalAttributes, uint32_t>());
    s_eem_attr_access->Init();

    // Accuracy is a mandatory attribute; publish a wide, coarse range.
    static const MeasurementAccuracyRangeStruct::Type ranges[] = {
        { .rangeMin = 0, .rangeMax = 1000000000000000,
          .percentMax = MakeOptional(static_cast<chip::Percent100ths>(500)),
          .percentMin = MakeOptional(static_cast<chip::Percent100ths>(50)) }
    };
    MeasurementAccuracyStruct::Type accuracy = {
        .measurementType  = MeasurementTypeEnum::kElectricalEnergy,
        .measured         = true,
        .minMeasuredValue = 0,
        .maxMeasuredValue = 1000000000000000,
        .accuracyRanges   = List<const MeasurementAccuracyRangeStruct::Type>(ranges, 1)
    };
    CHIP_ERROR err = SetMeasurementAccuracy(endpoint_id, accuracy);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "SetMeasurementAccuracy failed: %" CHIP_ERROR_FORMAT, err.Format());
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ElectricalEnergyMeasurement ready on endpoint %u", endpoint_id);
    return ESP_OK;
}

void energy_measurement_set_wh(uint16_t endpoint_id, int32_t total_wh)
{
    if (!s_eem_attr_access) {
        return;
    }
    EnergyMeasurementStruct::Type imported;
    imported.energy     = static_cast<int64_t>(total_wh) * 1000; // Wh -> mWh
    imported.endSystime = MakeOptional(static_cast<uint64_t>(System::SystemClock().GetMonotonicTimestamp().count()));

    Optional<EnergyMeasurementStruct::Type> opt_imported(imported);
    Optional<EnergyMeasurementStruct::Type> opt_exported; // none
    NotifyCumulativeEnergyMeasured(endpoint_id, opt_imported, opt_exported);
}
