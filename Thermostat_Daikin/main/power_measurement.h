#pragma once

// ElectricalPowerMeasurement (0x0090) delegate for exposing real-time power (W).
//
// esp-matter creates this cluster's attributes as MANAGED_INTERNALLY, so they
// can only be served by a connectedhomeip Instance + Delegate. When a delegate
// is set in the cluster config, esp-matter's ElectricalPowerMeasurementDelegateInitCB
// builds the Instance and calls Init() automatically (auto-matching the feature
// map), so all we provide here is the delegate. Trimmed from esp-matter's
// all_device_types_app electrical_measurement example (only ActivePower is
// populated; every other measurement returns Null).

#include <stdint.h>
#include <esp_err.h>
#include <app/clusters/electrical-power-measurement-server/electrical-power-measurement-server.h>

namespace chip {
namespace app {
namespace Clusters {
namespace ElectricalPowerMeasurement {

class ElectricalPowerMeasurementDelegate : public ElectricalPowerMeasurement::Delegate
{
public:
    ~ElectricalPowerMeasurementDelegate() = default;

    // --- attribute accessors (served to the AttributeAccessInterface) ---
    PowerModeEnum GetPowerMode() override { return mPowerMode; }
    uint8_t GetNumberOfMeasurementTypes() override;

    CHIP_ERROR StartAccuracyRead() override;
    CHIP_ERROR GetAccuracyByIndex(uint8_t, Structs::MeasurementAccuracyStruct::Type &) override;
    CHIP_ERROR EndAccuracyRead() override;

    CHIP_ERROR StartRangesRead() override;
    CHIP_ERROR GetRangeByIndex(uint8_t, Structs::MeasurementRangeStruct::Type &) override;
    CHIP_ERROR EndRangesRead() override;

    CHIP_ERROR StartHarmonicCurrentsRead() override;
    CHIP_ERROR GetHarmonicCurrentsByIndex(uint8_t, Structs::HarmonicMeasurementStruct::Type &) override;
    CHIP_ERROR EndHarmonicCurrentsRead() override;

    CHIP_ERROR StartHarmonicPhasesRead() override;
    CHIP_ERROR GetHarmonicPhasesByIndex(uint8_t, Structs::HarmonicMeasurementStruct::Type &) override;
    CHIP_ERROR EndHarmonicPhasesRead() override;

    DataModel::Nullable<int64_t> GetVoltage() override { return mVoltage; }
    DataModel::Nullable<int64_t> GetActiveCurrent() override { return mActiveCurrent; }
    DataModel::Nullable<int64_t> GetReactiveCurrent() override { return mReactiveCurrent; }
    DataModel::Nullable<int64_t> GetApparentCurrent() override { return mApparentCurrent; }
    DataModel::Nullable<int64_t> GetActivePower() override { return mActivePower; }
    DataModel::Nullable<int64_t> GetReactivePower() override { return mReactivePower; }
    DataModel::Nullable<int64_t> GetApparentPower() override { return mApparentPower; }
    DataModel::Nullable<int64_t> GetRMSVoltage() override { return mRMSVoltage; }
    DataModel::Nullable<int64_t> GetRMSCurrent() override { return mRMSCurrent; }
    DataModel::Nullable<int64_t> GetRMSPower() override { return mRMSPower; }
    DataModel::Nullable<int64_t> GetFrequency() override { return mFrequency; }
    DataModel::Nullable<int64_t> GetPowerFactor() override { return mPowerFactor; }
    DataModel::Nullable<int64_t> GetNeutralCurrent() override { return mNeutralCurrent; }

    // The only value we actually drive.
    CHIP_ERROR SetActivePower(DataModel::Nullable<int64_t>);

private:
    PowerModeEnum mPowerMode = PowerModeEnum::kUnknown;
    DataModel::Nullable<int64_t> mVoltage;
    DataModel::Nullable<int64_t> mActiveCurrent;
    DataModel::Nullable<int64_t> mReactiveCurrent;
    DataModel::Nullable<int64_t> mApparentCurrent;
    DataModel::Nullable<int64_t> mActivePower;
    DataModel::Nullable<int64_t> mReactivePower;
    DataModel::Nullable<int64_t> mApparentPower;
    DataModel::Nullable<int64_t> mRMSVoltage;
    DataModel::Nullable<int64_t> mRMSCurrent;
    DataModel::Nullable<int64_t> mRMSPower;
    DataModel::Nullable<int64_t> mFrequency;
    DataModel::Nullable<int64_t> mPowerFactor;
    DataModel::Nullable<int64_t> mNeutralCurrent;
};

} // namespace ElectricalPowerMeasurement
} // namespace Clusters
} // namespace app
} // namespace chip

// --- app-facing API ---
// Singleton delegate to hand to esp_matter's electrical_power_measurement config.
chip::app::Clusters::ElectricalPowerMeasurement::Delegate *power_measurement_delegate();
// Update ActivePower from the AC reading (watts). Call from the S21 callback.
void power_measurement_set_watts(int32_t watts);

// --- ElectricalEnergyMeasurement (CumulativeEnergyImported) ---
// Registers the energy AttributeAccessInterface + accuracy. Call ONCE, on the
// Matter thread, after esp_matter::start().
esp_err_t energy_measurement_init(uint16_t endpoint_id);
// Update CumulativeEnergyImported (lifetime total, Wh). Call on the Matter thread.
void energy_measurement_set_wh(uint16_t endpoint_id, int32_t total_wh);
