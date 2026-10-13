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

## Smooth Turning & Gap Navigation Stabilization
- **Silky-Smooth Differential Steering**: Removed abrupt discrete braking drops and inner-wheel reverse step thresholds. Inner wheel now decelerates smoothly towards zero with continuous PID output, completely eliminating corner twitches and violent jolts.
- **Calibrated 250Hz Control Loop**: Adjusted loop period to 4000 µs (250 Hz) to harmonize with DC motor mechanical response and BTS7960 slew rate.
- **Gentle Progressive PID & Low-Pass Derivative**: Softened cubic curve to 0.003 and increased D-filter smoothing (0.82/0.18), eliminating high-frequency derivative sensor chatter.
- **Straightforward Gap Trajectory Lock**: Completely removed premature 360° spinning in the middle of straight gaps. In `GapMode::Recover`, robot now holds steady forward cruising momentum with gentle trajectory tracking and seamlessly snaps back onto the line the moment it is detected.
## Kakarobot High-Speed Pro Profile (Competition Spec)
- **Zero Phase Lag (Pure PD Control)**: Configured $Ki = 0.0$ in the `FAST` speed profile to completely eliminate integral windup, steering wobble, and delayed phase drift at extreme velocity.
- **High Derivative Damping ($Kd = 3.80$)**: Strong derivative damping instantly arrests overshoot and wheel snap when tracking tight arcs at 2.5–3.5 m/s.
- **High-Velocity Wheel Authority**: Base speed elevated to 140 PWM, straight turbo ceiling to 210 PWM, and curve speed maintained at 105 PWM (75% momentum conservation) to prevent deceleration stall on curves.
## Ultra-High Accuracy & Tracking Upgrades
- **Baseline-Subtracted Gaussian Centroid**: Subtracted dynamic threshold baseline from raw sensor measurements in `LFR_Sensor.cpp`, amplifying true line peak signal-to-noise ratio and providing sub-millimeter centroid position resolution.
- **Dual-Zone Center Smoothing**: In `LFR_PID.cpp`, added smooth progressive center-zone tapering ($|error| < 0.35$) to eliminate straightaway micro-twitching while maintaining razor-sharp cubic exponential steering for cornering.
- **Radar Lookahead Corner Anticipation**: In `LFR_Control.cpp`, outer sensor engagement (0–1 or 12–13) immediately triggers anticipatory curve deceleration before centrifugal force can cause drift.
- **High-Precision 333Hz Control Loop**: Boosted control loop rate to 3000 µs (333 Hz), providing 33% more corrective feedback cycles per second.

## Tournament & Race Feature Upgrades
- **OLED Lap Timer & Race Telemetry**: `finishRace()` now accurately calculates and renders high-precision lap duration (`TIME 06.42s`), peak straightaway speed (`TOP PWM`), and junction crossings right on the SSD1306 OLED and Serial1.
- **Race Launch Control & 3-2-1 Countdown**: When Arming, a 3-second visual and audio countdown (`drawCountdown`) gives time to stand clear. Anti-slip launch control caps throttle at 75% for the first 140ms to maximize tire traction without wheel spin.
- **One-Click Auto-Sweep Calibration**: In the calibration menu, pressing UP (`UP AUTO`) triggers an automated 900ms left/right motor wiggle that dynamically scans all 14 channels over the live track, verifies contrast, and auto-saves calibration without needing separate papers.
- **Audio Chimes & Visual Feedback**: Integrated safe multi-tone frequency chimes (`soundChime`) with PC13 onboard LED blink synchronization for countdown, launch, success, error, and victory finish chimes. Supports optional buzzer on `LFR_PIN_BUZZER`.
