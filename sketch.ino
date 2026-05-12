#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <string.h>

#include "hvac_types.h"
#include "hvac_logic.h"

#define DHT_PIN             4
#define ADC_PIN             34
#define EMERGENCY_BTN_PIN   25
#define RESET_BTN_PIN       26

#define FAN_RELAY_PIN       16
#define HEATER_RELAY_PIN    17
#define IDLE_LED_PIN        23
#define FAULT_LED_PIN       18
#define BUZZER_PIN          19

/*
  Simulated 2-channel relay board:
  CH1 = Fan / cooling / ventilation
  CH2 = Heater

  The orange and blue LEDs are now connected directly to the same ESP32
  GPIO outputs that drive the relay inputs. This prevents NO/NC relay
  contact inversion from making the LEDs appear backwards.

  Required visible LED behaviour:
  Heating = orange LED only
  Cooling = blue LED only
  Idle = green LED only
  Emergency / overheat / fault = red LED only
*/
#define OUTPUT_ON           HIGH
#define OUTPUT_OFF          LOW

#define LCD_SDA_PIN         21
#define LCD_SCL_PIN         22

#define DHT_TYPE DHT22

#define SENSOR_TASK_PRIORITY       3
#define CONTROL_TASK_PRIORITY      4
#define SAFETY_TASK_PRIORITY       5
#define DIAGNOSTICS_TASK_PRIORITY  1
#define DISPLAY_TASK_PRIORITY      2

DHT dht_sensor(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, HVAC_LCD_COLS, HVAC_LCD_ROWS);

static SystemState g_system_state;
static SemaphoreHandle_t g_state_mutex = NULL;
static SemaphoreHandle_t g_serial_mutex = NULL;

volatile bool g_emergency_isr_flag = false;
volatile bool g_reset_isr_flag = false;
volatile uint32_t g_emergency_isr_time_ms = 0;
volatile uint32_t g_reset_isr_time_ms = 0;

static void serial_print_line(const char *message)
{
  if (g_serial_mutex != NULL && xSemaphoreTake(g_serial_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    Serial.println(message);
    xSemaphoreGive(g_serial_mutex);
  }
}

static void serial_print_header_once(void)
{
  serial_print_line("\n====================================================");
  serial_print_line("Industrial IoT Smart HVAC Controller - ESP32 Wokwi");
  serial_print_line("FreeRTOS + mutex + ISR buttons + ADC + DHT22 + LCD");
  serial_print_line("Simulated 2-channel relay: CH1 cooling/fan, CH2 heater");
  serial_print_line("Status LEDs are direct GPIO outputs to prevent relay contact inversion.");
  serial_print_line("LED rule: heating orange only, cooling blue only, idle green only, fault red only.");
  serial_print_line("Reset-required safety: lower temperature below 40C then press RESET.");
  serial_print_line("====================================================\n");
}

static void system_state_init(SystemState *state)
{
  if (state == NULL) return;

  memset(state, 0, sizeof(SystemState));

  state->temperature_c = 0.0f;
  state->humidity_percent = 0.0f;

  state->raw_adc = 0;
  state->adc_voltage = 0.0f;
  state->calibrated_adc_percent = 0.0f;
  state->filtered_adc_percent = 0.0f;
  state->adc_filter_ready = false;

  state->sensor_valid = false;
  state->sensor_stale = true;

  state->hvac_mode = HVAC_MODE_BOOT;
  state->fault_state = FAULT_NONE;
  state->emergency_active = false;

  state->heater_output = false;
  state->fan_output = false;
  state->idle_led_output = false;
  state->fault_led_output = false;
  state->buzzer_output = false;

  snprintf(
    state->last_safety_message,
    sizeof(state->last_safety_message),
    "Booting and waiting for first sensor read"
  );
}

static void update_outputs_from_decision(const HvacDecision *decision)
{
  if (decision == NULL) return;

  /*
    CH1 / GPIO16 / RX2 = fan relay input and blue cooling LED
    CH2 / GPIO17 / TX2 = heater relay input and orange heater LED

    Because LEDs are now direct GPIO status outputs:
    fan_on true = blue LED on
    heater_on true = orange LED on
  */
  digitalWrite(FAN_RELAY_PIN, decision->fan_on ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(HEATER_RELAY_PIN, decision->heater_on ? OUTPUT_ON : OUTPUT_OFF);

  digitalWrite(IDLE_LED_PIN, decision->idle_led_on ? OUTPUT_ON : OUTPUT_OFF);
  digitalWrite(FAULT_LED_PIN, decision->fault_led_on ? OUTPUT_ON : OUTPUT_OFF);

  if (decision->buzzer_on)
  {
    tone(BUZZER_PIN, 1600);
  }
  else
  {
    noTone(BUZZER_PIN);
  }
}

void IRAM_ATTR emergency_button_isr()
{
  g_emergency_isr_flag = true;
  g_emergency_isr_time_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
}

void IRAM_ATTR reset_button_isr()
{
  g_reset_isr_flag = true;
  g_reset_isr_time_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
}

void sensor_task(void *parameter)
{
  SystemState *state = (SystemState *)parameter;

  for (;;)
  {
    float temperature_c = dht_sensor.readTemperature();
    float humidity_percent = dht_sensor.readHumidity();

    uint16_t raw_adc = (uint16_t)analogRead(ADC_PIN);

    float adc_voltage = hvac_adc_to_voltage(raw_adc);
    float calibrated_adc = hvac_calibrate_adc_to_percent(raw_adc);

    bool values_ok = hvac_sensor_values_plausible(
      temperature_c,
      humidity_percent,
      raw_adc
    );

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      if (values_ok)
      {
        state->temperature_c = temperature_c;
        state->humidity_percent = humidity_percent;
      }

      state->raw_adc = raw_adc;
      state->adc_voltage = adc_voltage;
      state->calibrated_adc_percent = calibrated_adc;

      if (!state->adc_filter_ready)
      {
        state->filtered_adc_percent = calibrated_adc;
        state->adc_filter_ready = true;
      }
      else
      {
        state->filtered_adc_percent = hvac_exponential_filter(
          state->filtered_adc_percent,
          calibrated_adc,
          HVAC_ADC_FILTER_ALPHA
        );
      }

      state->sensor_valid = values_ok;
      state->sensor_stale = false;
      state->last_sensor_update_ms = millis();
      state->heartbeat_sensor_ms = millis();

      state->sensor_updates++;
      state->mutex_update_counter++;

      if (!values_ok)
      {
        snprintf(
          state->last_safety_message,
          sizeof(state->last_safety_message),
          "DHT22 invalid reading detected"
        );
      }

      xSemaphoreGive(g_state_mutex);

      if (values_ok)
      {
        serial_print_line("[sensor_task] Valid DHT22 and ADC values updated using mutex protection.");
      }
      else
      {
        serial_print_line("[sensor_task] Invalid DHT22 reading detected. ADC still updated using mutex protection.");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void control_task(void *parameter)
{
  SystemState *state = (SystemState *)parameter;
  HvacMode previous_mode = HVAC_MODE_BOOT;

  for (;;)
  {
    SystemState snapshot;
    bool snapshot_ok = false;

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      snapshot = *state;
      xSemaphoreGive(g_state_mutex);
      snapshot_ok = true;
    }

    if (snapshot_ok)
    {
      HvacDecision decision = hvac_make_decision(&snapshot);

      update_outputs_from_decision(&decision);

      if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
      {
        state->hvac_mode = decision.mode;
        state->heater_output = decision.heater_on;
        state->fan_output = decision.fan_on;
        state->idle_led_output = decision.idle_led_on;
        state->fault_led_output = decision.fault_led_on;
        state->buzzer_output = decision.buzzer_on;

        state->heartbeat_control_ms = millis();
        state->control_updates++;
        state->mutex_update_counter++;

        xSemaphoreGive(g_state_mutex);
      }

      if (decision.mode != previous_mode)
      {
        char msg[150];

        snprintf(
          msg,
          sizeof(msg),
          "[control_task] Mode changed to %s. GPIO LEDs and relay inputs updated.",
          hvac_mode_to_text(decision.mode)
        );

        serial_print_line(msg);
        previous_mode = decision.mode;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void safety_task(void *parameter)
{
  SystemState *state = (SystemState *)parameter;
  uint32_t last_accepted_emergency_ms = 0;
  uint32_t last_accepted_reset_ms = 0;

  for (;;)
  {
    bool emergency_event = false;
    bool reset_event = false;
    uint32_t emergency_event_time = 0;
    uint32_t reset_event_time = 0;

    noInterrupts();

    if (g_emergency_isr_flag)
    {
      emergency_event = true;
      emergency_event_time = g_emergency_isr_time_ms;
      g_emergency_isr_flag = false;
    }

    if (g_reset_isr_flag)
    {
      reset_event = true;
      reset_event_time = g_reset_isr_time_ms;
      g_reset_isr_flag = false;
    }

    interrupts();

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      uint32_t now = millis();

      if (emergency_event)
      {
        if ((emergency_event_time - last_accepted_emergency_ms) >= HVAC_BUTTON_DEBOUNCE_MS)
        {
          state->emergency_active = true;
          state->fault_state = FAULT_EMERGENCY_STOP;
          state->emergency_button_events++;
          state->last_emergency_press_ms = emergency_event_time;
          last_accepted_emergency_ms = emergency_event_time;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Emergency stop latched: press RESET after values are safe"
          );
        }
        else
        {
          state->ignored_button_events++;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Emergency button bounce/flood ignored"
          );
        }
      }

      if (reset_event)
      {
        if ((reset_event_time - last_accepted_reset_ms) >= HVAC_BUTTON_DEBOUNCE_MS)
        {
          state->reset_button_events++;
          state->last_reset_press_ms = reset_event_time;
          last_accepted_reset_ms = reset_event_time;

          if (hvac_values_safe_for_reset(state))
          {
            state->emergency_active = false;
            state->fault_state = FAULT_NONE;

            snprintf(
              state->last_safety_message,
              sizeof(state->last_safety_message),
              "Manual reset accepted: fault cleared and normal logic resumed"
            );
          }
          else
          {
            snprintf(
              state->last_safety_message,
              sizeof(state->last_safety_message),
              "Manual reset rejected: lower temperature below 40C and ensure sensor is valid"
            );
          }
        }
        else
        {
          state->ignored_button_events++;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Reset button bounce/flood ignored"
          );
        }
      }

      if (state->sensor_updates > 0 &&
          (now - state->last_sensor_update_ms) > HVAC_SENSOR_STALE_MS)
      {
        state->sensor_stale = true;

        if (state->fault_state == FAULT_NONE)
        {
          state->fault_state = FAULT_SENSOR_STALE;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Fault: sensor data stale"
          );
        }
      }

      if (!state->sensor_valid && state->sensor_updates > 0)
      {
        if (state->fault_state == FAULT_NONE)
        {
          state->fault_state = FAULT_SENSOR_INVALID;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Fault: invalid sensor values detected"
          );
        }
      }

      if (state->sensor_valid && state->temperature_c >= HVAC_TEMP_DANGER_C)
      {
        state->fault_state = FAULT_OVERHEAT;

        snprintf(
          state->last_safety_message,
          sizeof(state->last_safety_message),
          "Fault: overheat latched, outputs forced OFF, press RESET when safe"
        );
      }

      if (state->sensor_valid &&
          state->filtered_adc_percent > 98.0f &&
          state->temperature_c < HVAC_TEMP_HEAT_ON_C)
      {
        if (state->fault_state == FAULT_NONE)
        {
          state->fault_state = FAULT_ADC_ANOMALY;

          snprintf(
            state->last_safety_message,
            sizeof(state->last_safety_message),
            "Fault: ADC load anomaly compared with temperature"
          );
        }
      }

      if (now > 12000UL)
      {
        bool stale_sensor_task = (now - state->heartbeat_sensor_ms) > HVAC_TASK_STALE_MS;
        bool stale_control_task = (now - state->heartbeat_control_ms) > HVAC_TASK_STALE_MS;
        bool stale_display_task = (now - state->heartbeat_display_ms) > (HVAC_TASK_STALE_MS + 4000UL);
        bool stale_diagnostics_task = (now - state->heartbeat_diagnostics_ms) > (HVAC_TASK_STALE_MS + 4000UL);

        if (stale_sensor_task ||
            stale_control_task ||
            stale_display_task ||
            stale_diagnostics_task)
        {
          if (state->fault_state == FAULT_NONE)
          {
            state->fault_state = FAULT_TASK_HEARTBEAT;

            snprintf(
              state->last_safety_message,
              sizeof(state->last_safety_message),
              "Fault: task heartbeat stale"
            );
          }
        }
      }

      if (state->fault_state == FAULT_NONE &&
          !state->emergency_active &&
          state->sensor_valid &&
          !state->sensor_stale)
      {
        snprintf(
          state->last_safety_message,
          sizeof(state->last_safety_message),
          "System normal: sensors valid, no active safety fault"
        );
      }

      state->heartbeat_safety_ms = now;
      state->safety_updates++;
      state->mutex_update_counter++;

      xSemaphoreGive(g_state_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void diagnostics_task(void *parameter)
{
  SystemState *state = (SystemState *)parameter;

  for (;;)
  {
    SystemState snapshot;
    bool snapshot_ok = false;

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      state->heartbeat_diagnostics_ms = millis();
      state->diagnostics_updates++;
      state->mutex_update_counter++;

      snapshot = *state;

      xSemaphoreGive(g_state_mutex);
      snapshot_ok = true;
    }

    if (snapshot_ok &&
        g_serial_mutex != NULL &&
        xSemaphoreTake(g_serial_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      uint32_t now = millis();

      Serial.println("\n---------------- HVAC DIAGNOSTICS ----------------");

      Serial.printf(
        "Temperature: %.1f C | Humidity: %.1f %%\n",
        snapshot.temperature_c,
        snapshot.humidity_percent
      );

      Serial.printf(
        "ADC raw: %u / 4095 | ADC voltage: %.2f V\n",
        snapshot.raw_adc,
        snapshot.adc_voltage
      );

      Serial.printf(
        "ADC calibrated load: %.1f %% | ADC filtered load: %.1f %%\n",
        snapshot.calibrated_adc_percent,
        snapshot.filtered_adc_percent
      );

      Serial.printf(
        "Analogue vs digital comparison: load %.1f %% compared with DHT22 temp %.1f C\n",
        snapshot.filtered_adc_percent,
        snapshot.temperature_c
      );

      Serial.printf(
        "HVAC mode: %s | Fault: %s | Emergency: %s\n",
        hvac_mode_to_text(snapshot.hvac_mode),
        fault_state_to_text(snapshot.fault_state),
        snapshot.emergency_active ? "YES" : "NO"
      );

      Serial.printf(
        "Outputs -> fan/blue:%s heater/orange:%s idle/green:%s fault/red:%s buzzer:%s\n",
        snapshot.fan_output ? "ON" : "OFF",
        snapshot.heater_output ? "ON" : "OFF",
        snapshot.idle_led_output ? "ON" : "OFF",
        snapshot.fault_led_output ? "ON" : "OFF",
        snapshot.buzzer_output ? "ON" : "OFF"
      );

      Serial.printf(
        "Button events -> emergency:%lu reset:%lu ignored/flood:%lu\n",
        (unsigned long)snapshot.emergency_button_events,
        (unsigned long)snapshot.reset_button_events,
        (unsigned long)snapshot.ignored_button_events
      );

      Serial.printf(
        "Heartbeat ages ms -> sensor:%lu control:%lu safety:%lu display:%lu diagnostics:%lu\n",
        (unsigned long)(now - snapshot.heartbeat_sensor_ms),
        (unsigned long)(now - snapshot.heartbeat_control_ms),
        (unsigned long)(now - snapshot.heartbeat_safety_ms),
        (unsigned long)(now - snapshot.heartbeat_display_ms),
        (unsigned long)(now - snapshot.heartbeat_diagnostics_ms)
      );

      Serial.printf(
        "Task updates -> S:%lu C:%lu Safe:%lu D:%lu LCD:%lu\n",
        (unsigned long)snapshot.sensor_updates,
        (unsigned long)snapshot.control_updates,
        (unsigned long)snapshot.safety_updates,
        (unsigned long)snapshot.diagnostics_updates,
        (unsigned long)snapshot.display_updates
      );

      Serial.printf(
        "Mutex-protected shared updates: %lu\n",
        (unsigned long)snapshot.mutex_update_counter
      );

      Serial.printf("Safety decision: %s\n", snapshot.last_safety_message);
      Serial.println("--------------------------------------------------");

      xSemaphoreGive(g_serial_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(4000));
  }
}

void display_task(void *parameter)
{
  SystemState *state = (SystemState *)parameter;
  bool show_sensor_screen = true;

  for (;;)
  {
    SystemState snapshot;
    bool snapshot_ok = false;

    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      state->heartbeat_display_ms = millis();
      state->display_updates++;
      state->mutex_update_counter++;

      snapshot = *state;

      xSemaphoreGive(g_state_mutex);
      snapshot_ok = true;
    }

    if (snapshot_ok)
    {
      lcd.clear();

      if (show_sensor_screen)
      {
        lcd.setCursor(0, 0);

        if (snapshot.sensor_valid)
        {
          lcd.print("T:");
          lcd.print(snapshot.temperature_c, 1);
          lcd.print("C H:");
          lcd.print(snapshot.humidity_percent, 0);
          lcd.print("%");
        }
        else
        {
          lcd.print("DHT22 waiting...");
        }

        lcd.setCursor(0, 1);
        lcd.print("Load:");
        lcd.print(snapshot.filtered_adc_percent, 0);
        lcd.print("% ADC");
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Mode:");
        lcd.print(hvac_mode_to_text(snapshot.hvac_mode));

        lcd.setCursor(0, 1);

        if (snapshot.emergency_active)
        {
          lcd.print("RESET WHEN SAFE");
        }
        else if (snapshot.fault_state != FAULT_NONE)
        {
          lcd.print("FAULT RESET SAFE");
        }
        else
        {
          lcd.print("System Normal");
        }
      }

      show_sensor_screen = !show_sensor_screen;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  g_state_mutex = xSemaphoreCreateMutex();
  g_serial_mutex = xSemaphoreCreateMutex();

  system_state_init(&g_system_state);

  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(IDLE_LED_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(EMERGENCY_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(ADC_PIN, INPUT);

  digitalWrite(FAN_RELAY_PIN, OUTPUT_OFF);
  digitalWrite(HEATER_RELAY_PIN, OUTPUT_OFF);
  digitalWrite(IDLE_LED_PIN, OUTPUT_OFF);
  digitalWrite(FAULT_LED_PIN, OUTPUT_OFF);
  noTone(BUZZER_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  dht_sensor.begin();
  delay(2000);

  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart HVAC");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  attachInterrupt(
    digitalPinToInterrupt(EMERGENCY_BTN_PIN),
    emergency_button_isr,
    FALLING
  );

  attachInterrupt(
    digitalPinToInterrupt(RESET_BTN_PIN),
    reset_button_isr,
    FALLING
  );

  serial_print_header_once();

  xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, &g_system_state, SENSOR_TASK_PRIORITY, NULL, 1);
  xTaskCreatePinnedToCore(control_task, "control_task", 4096, &g_system_state, CONTROL_TASK_PRIORITY, NULL, 1);
  xTaskCreatePinnedToCore(safety_task, "safety_task", 4096, &g_system_state, SAFETY_TASK_PRIORITY, NULL, 1);
  xTaskCreatePinnedToCore(diagnostics_task, "diagnostics_task", 6144, &g_system_state, DIAGNOSTICS_TASK_PRIORITY, NULL, 1);
  xTaskCreatePinnedToCore(display_task, "display_task", 4096, &g_system_state, DISPLAY_TASK_PRIORITY, NULL, 1);

  serial_print_line("[setup] FreeRTOS tasks created. loop() left empty because tasks run the controller.");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}