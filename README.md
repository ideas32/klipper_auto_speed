# Klipper Auto Speed 
Klipper module for automatically calculating and validating your printer's maximum reliable acceleration and velocity.

## Improvements Over Original

This version of `klipper_auto_speed` has been fundamentally redesigned for reliability, safety, and accuracy.

*   **Extreme Reliability:** When the test gives a 'safe' output acceleration, it is now highly trustworthy. The new multi-sample approach eliminates the overestimation common on the original.
*   **Inherently Safe Move Pattern:** Like the original, test moves are safe. The move distance is mathematically capped to ensure that even a complete failure (e.g., lost steps causing a move to continue in one direction) **cannot result in a collision with the printer's physical limits.**
*   **Comprehensive Testing:** Instead of a single, simple test, each acceleration value is now subjected to a gauntlet of **6 distinct test runs** (or more, user-configurable). This includes short, sharp "acceleration-limited" moves, and longer "velocity-plateau" moves, ensuring the final value is robust across different printing conditions.
*   **Automated Confidence:** The script automatically concludes with a **high-iteration safe validation run** (default 30 consecutive tests) to provide extremely high confidence that the final recommended value is stable and repeatable.
*   **Intelligent, Physics-Based Workflow:** The primary `AUTO_SPEED` command now follows a logical, multi-stage process:
    1.  First, it finds the maximum acceleration in the "constant-torque" region of the stepper motors (at a safe, low speed).
    2.  Next, it uses that known-safe acceleration to find the printer's true maximum velocity.
    3.  Finally, it characterizes and graphs the machine's performance at even higher speeds.

> **Disclaimer:** This is an unofficial modification. The code is provided as-is, without warranty or liability. Always be prepared to stop your printer during any test. Use at your own risk.

---

## How Does It Work?

The new `AUTO_SPEED` command is a fully automated, multi-stage routine.

1.  **Preparation**
    *   The printer is homed and leveled (if applicable).
    *   A quick check is run to ensure your endstops are reasonably consistent.

2.  **Stage 1: Find Baseline Max Acceleration**
    *   The script performs a binary search for the highest possible acceleration your printer can handle at a safe, low speed (e.g., 200 mm/s).
    *   For **each** acceleration value it tests, it runs a gauntlet of tests using the safe, centered move pattern:
        *   3x "Short & Sharp" moves (pure accel/decel).
        *   3x "Long & Smooth" moves (accel/coast/decel).
    *   An acceleration value must pass all 6 tests to be considered valid. This process uses an efficient "chained homing" method to maintain safety while reducing test time.

3.  **Stage 2: Find Max Velocity**
    *   Using the safe, derated acceleration found in Stage 1, the script runs a second binary search to find the absolute maximum velocity the printer can achieve.

4.  **Stage 3: Performance Characterization (Graphing)**
    *   The script automatically runs a series of tests to see how the maximum acceleration changes at higher velocities, plotting the results to a graph for your review.

5.  **Stage 4: Automated Safe Validation**
    *   The script takes the final recommended acceleration value from Stage 1.
    *   It then runs a high number of validation tests (e.g., 30) using the safe, centered move pattern.
    *   **Each of these 30 tests performs a full, independent homing cycle** for maximum safety, confirming that the recommended value is not just fast, but exceptionally reliable.

---

## Using Klipper Auto Speed

### Installation
(Installation instructions remain the same as the original)

### Configuration
Place the following in your `printer.cfg`. The values shown are the defaults; you only need to add the ones you wish to change.

```ini
[auto_speed]
# --- Primary Configuration ---
# The main AUTO_SPEED command will test the first axis listed here.
# One or multiple of `x`, `y`, `diag_x`, `diag_y`, `z`.
axis: diag_x, diag_y

# Derate discovered results by this amount for a safety margin. 0.8 = 80%.
derate: 0.8

# --- NEW: Parameters for the Robust Test Engine ---
# The "safe" velocity (mm/s) to use when finding the baseline acceleration.
# Should be well within your motor's constant-torque range. 200 is a very safe default.
accel_test_velocity: 200

# The number of samples for EACH test type (short and long) in the search.
# SAMPLES=3 means 3 short + 3 long = 6 total tests per acceleration value.
samples_per_test_type: 3

# The number of iterations for the final automated safety validation run.
final_validation_iterations: 30

# --- Search Bounds ---
accel_min: 1000.0
accel_max: 50000.0
velocity_max: 5000.0

# --- Legacy Parameters ---
# Note: The new robust accel test uses a hardcoded physics-based threshold (3.0 steps).
# The 'max_missed' parameter is now only used by the legacy _VELOCITY and _GRAPH commands.
max_missed: 1.0 
```

### Macros

The `AUTO_SPEED` command has been promoted to the primary, all-in-one tool. Other commands are now for specialized or legacy use.

#### **`AUTO_SPEED` (Recommended)**
This is the main event. It runs the full, multi-stage characterization and validation routine.

**Usage:**
```gcode
# Run the complete, end-to-end test with settings from your config file.
AUTO_SPEED

# Override the axis and number of samples for a specific run.
AUTO_SPEED AXIS=x SAMPLES=5
```
**Arguments:**

| Argument | Default | Description |
|---|---|---|
| AXIS | (from config) | The primary axis to test. |
| ACCEL_MIN | (from config) | The lowest acceleration to test. |
| ACCEL_MAX | (from config) | The highest acceleration to test. |
| VELOCITY_MAX | (from config) | The highest velocity to test. |
| DERATE | (from config) | The safety derating factor (e.g., 0.8). |
| SAMPLES | (from config) | Overrides `samples_per_test_type`. |
| VALIDATION_ITERATIONS | (from config) | Overrides `final_validation_iterations`. |

---
#### **(Advanced / Legacy Commands)**

These commands can be used for specific, focused tests but are not part of the primary recommended workflow.

*   **`AUTO_SPEED_ACCEL`**: Runs *only* the new, robust acceleration search and its validation phase.
*   **`AUTO_SPEED_VELOCITY`**: Runs *only* the original, less-reliable velocity test. Requires an `ACCEL` parameter.
*   **`AUTO_SPEED_GRAPH`**: Runs *only* the original graphing routine.
*   **`AUTO_SPEED_VALIDATE`**: Runs the original test pattern from Ellis. **WARNING: This pattern is NOT inherently safe and should be supervised closely.**

---
## Console Output

The output is now broken into clear stages.

**Stage 1: Acceleration Search**
```
--- Testing accel value: 15000 ---
Initializing gauntlet with a starting home...
--- Running 3 Accel-Focused Tests (Short & Sharp) ---
Sample 1/3...
(Homing and move messages)
Sample 2/3...
(Homing and move messages)
Sample 3/3...
(Homing and move messages)
--- Running 3 Velo-Plateau Tests (Long & Smooth) ---
Sample 1/3...
(Homing and move messages)
...
VALUE 15000 PASSED all tests.
```

**Stage 4: Final Validation**
```
========================================
STAGE 4: Performing final safe validation of recommended accel (12000mm/s^2).
========================================

Validation run 1/30...
(Homing and move messages)
Validation run 2/30...
(Homing and move messages)
...
Validation run 30/30...
(Homing and move messages)

========================================
--- Automated Validation Complete ---
Result: 30 passes, 0 failures out of 30 runs.
CONFIRMATION PASSED: Recommended acceleration is highly reliable.
========================================
```
