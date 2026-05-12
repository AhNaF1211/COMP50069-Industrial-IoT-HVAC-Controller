#include "hvac_logic.h"
#include <math.h>

const char *hvac_mode_to_text(HvacMode mode)
{
  switch (mode)
  {
    case HVAC_MODE_BOOT:        return "BOOT";
    case HVAC_MODE_IDLE:        return "IDLE/OFF";
    case HVAC_MODE_HEATING:     return "HEATING";
    case HVAC_MODE_COOLING:     return "COOLING";
    case HVAC_MODE_VENTILATION: return "VENT";
    case HVAC_MODE_FAULT:       return "FAULT";
    case HVAC_MODE_EMERGENCY:   return "EMERGENCY";
    default:                    return "UNKNOWN";
  }
}

const char *fault_state_to_text(FaultState fault)
{
  switch (fault)
  {
    case FAULT_NONE:            return "NONE";
    case FAULT_SENSOR_INVALID:  return "SENSOR_INVALID";
    case FAULT_SENSOR_STALE:    return "SENSOR_STALE";
    case FAULT_OVERHEAT:        return "OVERHEAT";
    case FAULT_EMERGENCY_STOP:  return "EMERGENCY_STOP";
    case FAULT_TASK_HEARTBEAT:  return "TASK_HEARTBEAT";
    case FAULT_ADC_ANOMALY:     return "ADC_ANOMALY";
    default:                    return "UNKNOWN";
  }
}

float hvac_adc_to_voltage(uint16_t raw_adc)
{
  return ((float)raw_adc * HVAC_ADC_VREF_VOLTS) / HVAC_ADC_MAX_COUNTS;
}

float hvac_calibrate_adc_to_percent(uint16_t raw_adc)
{
  float voltage = hvac_adc_to_voltage(raw_adc);
  float percent = (voltage / HVAC_ADC_VREF_VOLTS) * 100.0f;

  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  return percent;
}

float hvac_exponential_filter(float previous_value, float new_value, float alpha)
{
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  return (alpha * new_value) + ((1.0f - alpha) * previous_value);
}

bool hvac_sensor_values_plausible(float temperature_c, float humidity_percent, uint16_t raw_adc)
{
  if (isnan(temperature_c) || isnan(humidity_percent)) return false;

  if (temperature_c < HVAC_MIN_REALISTIC_TEMP_C) return false;
  if (temperature_c > HVAC_MAX_REALISTIC_TEMP_C) return false;

  if (humidity_percent < HVAC_MIN_REALISTIC_HUMIDITY) return false;
  if (humidity_percent > HVAC_MAX_REALISTIC_HUMIDITY) return false;

  if (raw_adc > 4095) return false;

  return true;
}

bool hvac_values_safe_for_reset(const SystemState *state)
{
  if (state == NULL) return false;
  if (!state->sensor_valid) return false;
  if (state->sensor_stale) return false;
  if (state->temperature_c >= HVAC_TEMP_RESET_SAFE_C) return false;
  if (state->filtered_adc_percent >= 95.0f) return false;

  return true;
}

HvacDecision hvac_make_decision(const SystemState *state)
{
  HvacDecision decision;

  decision.heater_on = false;
  decision.fan_on = false;
  decision.idle_led_on = false;
  decision.fault_led_on = false;
  decision.buzzer_on = false;
  decision.mode = HVAC_MODE_IDLE;

  if (state == NULL)
  {
    decision.mode = HVAC_MODE_FAULT;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = true;
    decision.buzzer_on = true;
    return decision;
  }

  if (state->sensor_updates == 0 || !state->adc_filter_ready)
  {
    decision.mode = HVAC_MODE_BOOT;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = false;
    decision.buzzer_on = false;
    return decision;
  }

  if (state->emergency_active)
  {
    decision.mode = HVAC_MODE_EMERGENCY;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = true;
    decision.buzzer_on = true;
    return decision;
  }

  if (state->fault_state != FAULT_NONE)
  {
    decision.mode = HVAC_MODE_FAULT;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = true;
    decision.buzzer_on = true;
    return decision;
  }

  if (!state->sensor_valid || state->temperature_c >= HVAC_TEMP_DANGER_C)
  {
    decision.mode = HVAC_MODE_FAULT;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = true;
    decision.buzzer_on = true;
    return decision;
  }

  if (state->temperature_c <= HVAC_TEMP_HEAT_ON_C)
  {
    decision.mode = HVAC_MODE_HEATING;
    decision.heater_on = true;
    decision.fan_on = false;
    decision.idle_led_on = false;
    decision.fault_led_on = false;
    decision.buzzer_on = false;
  }
  else if (state->temperature_c >= HVAC_TEMP_COOL_ON_C)
  {
    decision.mode = HVAC_MODE_COOLING;
    decision.heater_on = false;
    decision.fan_on = true;
    decision.idle_led_on = false;
    decision.fault_led_on = false;
    decision.buzzer_on = false;
  }
  else if (state->filtered_adc_percent >= HVAC_ANALOG_HIGH_LOAD_PERCENT)
  {
    decision.mode = HVAC_MODE_VENTILATION;
    decision.heater_on = false;
    decision.fan_on = true;
    decision.idle_led_on = false;
    decision.fault_led_on = false;
    decision.buzzer_on = false;
  }
  else
  {
    decision.mode = HVAC_MODE_IDLE;
    decision.heater_on = false;
    decision.fan_on = false;
    decision.idle_led_on = true;
    decision.fault_led_on = false;
    decision.buzzer_on = false;
  }

  return decision;
}