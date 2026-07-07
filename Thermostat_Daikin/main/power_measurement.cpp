// ElectricalPowerMeasurement delegate — trimmed from esp-matter's
// all_device_types_app electrical_measurement example. Only ActivePower is
// driven from the AC; the remaining measurement accessors return their default
// (Null) values. The connectedhomeip Instance is built for us by esp-matter's
// ElectricalPowerMeasurementDelegateInitCB when this delegate is set in the
// cluster config.

#include "power_measurement.h"

#include <esp_log.h>
#include <app/reporting/reporting.h>
#include <app/data-model/Nullable.h>
#include <app/data-model/List.h>
#include <app-common/zap-generated/ids/Attributes.h>

using namespace chip;
using namespace chip::app;
using namespace chip::app::DataModel;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::ElectricalPowerMeasurement;
using namespace chip::app::Clusters::ElectricalPowerMeasurement::Attributes;
using namespace chip::app::Clusters::ElectricalPowerMeasurement::Structs;

static const char *TAG = "power_measurement";

namespace chip {
namespace app {
namespace Clusters {
namespace ElectricalPowerMeasurement {

// Accuracy ranges (from the reference example).
static const MeasurementAccuracyRangeStruct::Type activePowerAccuracyRanges[] = {
    { .rangeMin = -50'000'000, .rangeMax = -10'000'000,
      .percentMax = MakeOptional(static_cast<chip::Percent100ths>(5000)),
      .percentMin = MakeOptional(static_cast<chip::Percent100ths>(2000)),
      .percentTypical = MakeOptional(static_cast<chip::Percent100ths>(3000)) },
    { .rangeMin = -9'999'999, .rangeMax = 9'999'999,
      .percentMax = MakeOptional(static_cast<chip::Percent100ths>(1000)),
      .percentMin = MakeOptional(static_cast<chip::Percent100ths>(100)),
      .percentTypical = MakeOptional(static_cast<chip::Percent100ths>(500)) },
    { .rangeMin = 10'000'000, .rangeMax = 50'000'000,
      .percentMax = MakeOptional(static_cast<chip::Percent100ths>(5000)),
      .percentMin = MakeOptional(static_cast<chip::Percent100ths>(2000)),
      .percentTypical = MakeOptional(static_cast<chip::Percent100ths>(3000)) },
};

static const Structs::MeasurementAccuracyStruct::Type kMeasurementAccuracies[] = {
    { .measurementType = MeasurementTypeEnum::kActivePower, .measured = true,
      .minMeasuredValue = -50'000'000, .maxMeasuredValue = 50'000'000,
      .accuracyRanges = DataModel::List<const MeasurementAccuracyRangeStruct::Type>(activePowerAccuracyRanges) },
};

static const Structs::HarmonicMeasurementStruct::Type kHarmonicMeasurements[] = {
    { .order = 1, .measurement = MakeNullable(static_cast<int64_t>(100000)) }
};

uint8_t ElectricalPowerMeasurementDelegate::GetNumberOfMeasurementTypes()
{
    return MATTER_ARRAY_SIZE(kMeasurementAccuracies);
}

CHIP_ERROR ElectricalPowerMeasurementDelegate::StartAccuracyRead() { return CHIP_NO_ERROR; }
CHIP_ERROR ElectricalPowerMeasurementDelegate::GetAccuracyByIndex(uint8_t index, Structs::MeasurementAccuracyStruct::Type &accuracy)
{
    if (index >= MATTER_ARRAY_SIZE(kMeasurementAccuracies))
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    accuracy = kMeasurementAccuracies[index];
    return CHIP_NO_ERROR;
}
CHIP_ERROR ElectricalPowerMeasurementDelegate::EndAccuracyRead() { return CHIP_NO_ERROR; }

CHIP_ERROR ElectricalPowerMeasurementDelegate::StartRangesRead() { return CHIP_NO_ERROR; }
CHIP_ERROR ElectricalPowerMeasurementDelegate::GetRangeByIndex(uint8_t, Structs::MeasurementRangeStruct::Type &)
{
    return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
}
CHIP_ERROR ElectricalPowerMeasurementDelegate::EndRangesRead() { return CHIP_NO_ERROR; }

CHIP_ERROR ElectricalPowerMeasurementDelegate::StartHarmonicCurrentsRead() { return CHIP_NO_ERROR; }
CHIP_ERROR ElectricalPowerMeasurementDelegate::GetHarmonicCurrentsByIndex(uint8_t index, Structs::HarmonicMeasurementStruct::Type &h)
{
    if (index >= MATTER_ARRAY_SIZE(kHarmonicMeasurements))
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    h = kHarmonicMeasurements[index];
    return CHIP_NO_ERROR;
}
CHIP_ERROR ElectricalPowerMeasurementDelegate::EndHarmonicCurrentsRead() { return CHIP_NO_ERROR; }

CHIP_ERROR ElectricalPowerMeasurementDelegate::StartHarmonicPhasesRead() { return CHIP_NO_ERROR; }
CHIP_ERROR ElectricalPowerMeasurementDelegate::GetHarmonicPhasesByIndex(uint8_t index, Structs::HarmonicMeasurementStruct::Type &h)
{
    if (index >= MATTER_ARRAY_SIZE(kHarmonicMeasurements))
        return CHIP_ERROR_PROVIDER_LIST_EXHAUSTED;
    h = kHarmonicMeasurements[index];
    return CHIP_NO_ERROR;
}
CHIP_ERROR ElectricalPowerMeasurementDelegate::EndHarmonicPhasesRead() { return CHIP_NO_ERROR; }

CHIP_ERROR ElectricalPowerMeasurementDelegate::SetActivePower(DataModel::Nullable<int64_t> newValue)
{
    DataModel::Nullable<int64_t> oldValue = mActivePower;
    mActivePower = newValue;
    if (oldValue != newValue) {
        MatterReportingAttributeChangeCallback(mEndpointId, ElectricalPowerMeasurement::Id, ActivePower::Id);
    }
    return CHIP_NO_ERROR;
}

} // namespace ElectricalPowerMeasurement
} // namespace Clusters
} // namespace app
} // namespace chip

// --- app-facing API ---

static chip::app::Clusters::ElectricalPowerMeasurement::ElectricalPowerMeasurementDelegate s_epm_delegate;

chip::app::Clusters::ElectricalPowerMeasurement::Delegate *power_measurement_delegate()
{
    return &s_epm_delegate;
}

void power_measurement_set_watts(int32_t watts)
{
    // ActivePower is reported in milliwatts.
    s_epm_delegate.SetActivePower(chip::app::DataModel::MakeNullable(static_cast<int64_t>(watts) * 1000));
}
