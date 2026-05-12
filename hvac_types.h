#ifndef HVAC_TYPES_H
#define HVAC_TYPES_H

#include <Arduino.h>



#define HVAC_ADC_MAX_COUNTS             4095.0f
#define HVAC_ADC_VREF_VOLTS             3.30f
#define HVAC_ADC_FILTER_ALPHA           0.25f


#define HVAC_TEMP_HEAT_ON_C             18.0f
#define HVAC_TEMP_HEAT_OFF_C            20.0f
#define HVAC_TEMP_COOL_ON_C             26.0f
#define HVAC_TEMP_DANGER_C              42.0f

#define HVAC_TEMP_RESET_SAFE_C          40.0f

#define HVAC_MIN_REALISTIC_TEMP_C       -10.0f
#define HVAC_MAX_REALISTIC_TEMP_C       60.0f
#define HVAC_MIN_REALISTIC_HUMIDITY     0.0f
#define HVAC_MAX_REALISTIC_HUMIDITY     100.0f

#define HVAC_ANALOG_HIGH_LOAD_PERCENT   75.0f

#define HVAC_SENSOR_STALE_MS            15000UL
#define HVAC_TASK_STALE_MS              12000UL
#define HVAC_BUTTON_DEBOUNCE_MS         350UL

#define HVAC_LCD_COLS                   16
#define HVAC_LCD_ROWS                   2

typedef enum
{
  HVAC_MODE_BOOT = 0,
  HVAC_MODE_IDLE,
  HVAC_MODE_HEATING,
  HVAC_MODE_COOLING,
  HVAC_MODE_VENTILATION,
  HVAC_MODE_FAULT,
  HVAC_MODE_EMERGENCY
} HvacMode;


typedef enum
{
  FAULT_NONE = 0,
  FAULT_SENSOR_INVALID,
  FAULT_SENSOR_STALE,
  FAULT_OVERHEAT,
  FAULT_EMERGENCY_STOP,
  FAULT_TASK_HEARTBEAT,
  FAULT_ADC_ANOMALY
} FaultState;


typedef struct
{
  bool heater_on;
  bool fan_on;
  bool idle_led_on;
  bool fault_led_on;
  bool buzzer_on;
  HvacMode mode;
} HvacDecision;

typedef struct
{
  float temperature_c;
  float humidity_percent;

  uint16_t raw_adc;
  float adc_voltage;
  float calibrated_adc_percent;
  float filtered_adc_percent;
  bool adc_filter_ready;

  bool sensor_valid;
  bool sensor_stale;

  HvacMode hvac_mode;
  FaultState fault_state;
  bool emergency_active;

  bool heater_output;
  bool fan_output;
  bool idle_led_output;
  bool fault_led_output;
  bool buzzer_output;

  uint32_t emergency_button_events;
  uint32_t reset_button_events;
  uint32_t ignored_button_events;

  uint32_t sensor_updates;
  uint32_t control_updates;
  uint32_t safety_updates;
  uint32_t diagnostics_updates;
  uint32_t display_updates;
  uint32_t mutex_update_counter;

  uint32_t heartbeat_sensor_ms;
  uint32_t heartbeat_control_ms;
  uint32_t heartbeat_safety_ms;
  uint32_t heartbeat_diagnostics_ms;
  uint32_t heartbeat_display_ms;

  uint32_t last_sensor_update_ms;
  uint32_t last_emergency_press_ms;
  uint32_t last_reset_press_ms;

  char last_safety_message[96];
} SystemState;

#endif