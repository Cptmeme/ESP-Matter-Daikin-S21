#include "ac_manager.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "AC_MANAGER";

ACManager::ACManager()
    : m_count(0), m_tx_pin(-1), m_rx_pin(-1),
      m_active(nullptr), m_next_probe(0), m_callback(nullptr) {
    // Sensible defaults reported until a backend connects.
    m_default_state.power = false;
    m_default_state.mode = FAIKIN_MODE_AUTO;
    m_default_state.target_temp = 22.0f;
    m_default_state.current_temp = 21.0f;
    m_default_state.outside_temp = 0.0f;
    m_default_state.fan_speed = FAIKIN_FAN_AUTO;
    m_default_state.powerful = false;
    m_default_state.power_w = -1;
    m_default_state.energy_total_wh = -1;
    m_default_state.energy_cool_wh = -1;
    m_default_state.energy_heat_wh = -1;
}

void ACManager::AddBackend(ACBackend *b) {
    if (b && m_count < AC_MAX_BACKENDS) {
        m_backends[m_count++] = b;
    }
}

esp_err_t ACManager::Init(int tx_pin, int rx_pin) {
    m_tx_pin = tx_pin;
    m_rx_pin = rx_pin;
    m_active = nullptr;
    m_next_probe = 0;
    return ESP_OK;
}

void ACManager::Poll() {
    if (m_active) {
        m_active->Poll();
        // A backend may drop offline; if it does, fall back to re-detection.
        if (!m_active->IsConnected()) {
            ESP_LOGW(TAG, "%s link lost, will re-detect", m_active->Name());
            m_active->Deinit();
            m_active = nullptr;
            m_next_probe = 0;
        }
        return;
    }

    if (m_count == 0) {
        return;
    }

    // Probe one candidate per call so we never hold the task for long.
    ACBackend *cand = m_backends[m_next_probe];
    ESP_LOGI(TAG, "Probing protocol: %s", cand->Name());

    if (cand->Init(m_tx_pin, m_rx_pin) == ESP_OK && cand->Detect()) {
        ESP_LOGI(TAG, "AC protocol detected: %s", cand->Name());
        cand->SetStateCallback(m_callback);
        m_active = cand;
        return;
    }

    cand->Deinit();
    m_next_probe = (m_next_probe + 1) % m_count;
}

bool ACManager::IsConnected() const {
    return m_active && m_active->IsConnected();
}

const char *ACManager::ActiveName() const {
    return m_active ? m_active->Name() : "none";
}

void ACManager::SetPower(bool on)      { if (m_active) m_active->SetPower(on); }
void ACManager::SetMode(uint8_t mode)  { if (m_active) m_active->SetMode(mode); }
void ACManager::SetTemp(float temp)    { if (m_active) m_active->SetTemp(temp); }
void ACManager::SetFan(uint8_t fan)    { if (m_active) m_active->SetFan(fan); }
void ACManager::SetPowerful(bool on)   { if (m_active) m_active->SetPowerful(on); }

ac_state_t ACManager::GetState() const {
    return m_active ? m_active->GetState() : m_default_state;
}

void ACManager::SetStateCallback(s21_state_change_cb_t cb) {
    m_callback = cb;
    if (m_active) m_active->SetStateCallback(cb);
}
