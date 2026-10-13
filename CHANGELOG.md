# LFR PRO V3.1 OLED UI Fix — changes

- Dedicated 5x7 seven-segment digits for menu index, counters, and enlarged tuning values (with recognizable `1`).
- Aligned fixed two-digit menu indexes 01-15; modes displayed inline on the menu.
- New transactional ModeSelect screen: current vs candidate, UP/DOWN to preview, ENTER to apply, BACK to cancel.
- Speed profile now loads only after confirmation; choosing it loads the original V3 PWM/PID presets as before.
- New confirmation page for RESET DEFAULTS; keeps calibration unless actively rescanned.
- Cleaner PID numeric page and consistent footer; no change to PID, calibration, motor, timing, sensor, navigation, or EEPROM layout.
- Regression-tested UI, PID, lost-line safeguards, firmware setup, BACK emergency stop, and twenty-one rendered OLED screens.

## Bug Fixes & Reliability Upgrades
- **Finish Line Detection**: Fixed finish line crossing logic to detect standard 2–5 cm finish markers at high speed (3+ frame debounce) instead of waiting for an impossible 190ms dwell time.
- **Start Line Box Support**: Fixed false "START LINE INVALID" error when placed on a wide starting box or full start line.
- **Motor Glitch Elimination**: Removed redundant dual-zero write in `outputOne()`, eliminating 5ms switching PWM stutter, acoustic whine, and torque loss.
- **PB3 BACK E-Stop JTAG Clock**: Enabled AFIO clock before remapping JTAG in `setup()`, guaranteeing PB3 operates reliably as a GPIO button.
- **Corner vs. Junction Disambiguation**: Differentiated single-blob 90° corners from true junctions (`leftEdge && rightEdge` or branching line with `groups >= 2`), preventing the robot from driving straight off 90° turns.
- **Gap Speed Maintenance**: In Gap recovery, forward speed now maintains average momentum instead of dropping to crawl speed.
- **STM32 Core 3.0.0 Compatibility**: Added `STM32_CORE_VERSION` guard for standard `EEPROM.put()`, ensuring clean build on official ST Core.

## High-Performance Competition Upgrades (Pro-Level)
- **Straightaway Turbo Boost**: Automatically detects sustained straight tracking (low error/rate for >=12 frames) and ramps speed dynamically up toward `maxPWM` (up to 220 PWM).
- **Pre-Braking Assistance**: On rising track curvature (>0.45), hard braking engages immediately to safely decelerate to `curvePWM` before entering turns.
- **High-Rate 400Hz Control Loop**: Control loop interval reduced to 2500 µs (400 Hz), doubling real-time steering response and line lock.
- **Fast MUX ADC Sampling**: Multiplexer settling delay optimized to 6 µs per channel with single conversion, slashing 14-channel read time from ~1.8 ms to ~0.49 ms.
- **Non-Linear Progressive PID**: Cubic progressive gain ($error + 0.012 \times error^3$) ensures ultra-smooth straight running with aggressive steering torque at the track edges.
- **Corner Damping & Active Counter-Torque**: Added rotational momentum damping pulse on pivot exit to stop oscillation, and active inner-wheel counter-spin on severe turns ($|error| > 4.2$).
- **Adaptive Contrast Auto-Thresholding**: Dynamically computes threshold based on real-time background and line contrast (35% above noise floor), rendering the robot immune to track lighting fluctuations.
- **Plotter-Friendly Telemetry**: Compact 20Hz Serial1 telemetry format (`P:<pos> L:<left> R:<right> N:<active>`) for real-time Bluetooth graphing.


