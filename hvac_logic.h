#ifndef HVAC_LOGIC_H
#define HVAC_LOGIC_H

#include "hvac_types.h"


const char *hvac_mode_to_text(HvacMode mode);
const char *fault_state_to_text(FaultState fault);

float hvac_adc_to_voltage(uint16_t raw_adc);
float hvac_calibrate_adc_to_percent(uint16_t raw_adc);
float hvac_exponential_filter(float previous_value, float new_value, float alpha);

bool hvac_sensor_values_plausible(float temperature_c, float humidity_percent, uint16_t raw_adc);
bool hvac_values_safe_for_reset(const SystemState *state);

HvacDecision hvac_make_decision(const SystemState *state);

#endif