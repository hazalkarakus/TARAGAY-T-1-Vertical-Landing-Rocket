V55 USART2 TELEMETRY

TX  PA2
RX  PA3 (configured electrically; command parser disabled)
115200 baud, 8 data bits, no parity, 1 stop bit
10 Hz DMA transmission

Run:
  py -m pip install pyserial
  py monitor_uart_v55.py --port COM18

Frame prefix:
  $TGY55

Important interlock fields:
  preflight_state
  flight_active
  pe9_raw_open
  pe9_debounced_open
  connector_seen
  actuator_authorized
  rcs_req_mask
  rcs_applied_mask
  rcs_flight_authorized
  rcs_safety_inhibited
  needle_enabled
  needle_rpwm
  needle_lpwm
  needle_zero_valid
  needle_fault

Connector installed expectation:
  pe9_raw_open=0, flight_active=0, actuator_authorized=0,
  rcs_applied_mask=0, needle_enabled=0, needle_rpwm=0, needle_lpwm=0

PE9 electrical polarity:
  connector installed = PE9 shorted to GND / LOW
  connector removed   = PE9 open / internal pull-up / HIGH

Connector removed expectation, only after preflight_state=2:
  pe9_raw_open=1, flight_active=1, actuator_authorized=1

UART is deliberately read-only. It cannot start the motor or a solenoid.

Final relay wiring:
  IN1 = PB15 = X+
  IN2 = PE15 = X-
  IN3 = PE11 = Y+
  IN4 = PE7  = Y-
