# Agent Guide for openpilot

This document provides essential information for AI agents to effectively contribute to the openpilot/sunnypilot repository. openpilot is a **safety-critical system** - adherence to these guidelines is mandatory.

---

## Commands

### Build
openpilot uses SCons for building C++ and Cython code.
```bash
scons -j$(nproc)          # Full build (uses half of available cores by default)
scons -c                  # Clean build artifacts
scons -j$(nproc) --minimal # Minimal build (no tests/tools)
scons --asan              # Build with AddressSanitizer
scons --ubsan             # Build with UndefinedBehaviorSanitizer
```

### Linting & Type Checking
```bash
scripts/lint/lint.sh       # Run all linters (ruff, mypy, codespell, etc.)
scripts/lint/lint.sh -f    # Fast mode (skip mypy/codespell)
ruff check .               # Lint only
ruff format .              # Format only
mypy .                     # Type check only
codespell                  # Spell check only
```

### Testing
We use pytest with parallel execution by default (`-n auto`).
```bash
pytest                                              # Run all tests (parallel)
pytest path/to/test_file.py                         # Run single test file
pytest path/to/test_file.py -k test_name            # Run specific test by name
pytest path/to/test_file.py::TestClass::test_method # Run exact test
pytest -s path/to/test_file.py                      # Run with stdout visible
pytest -x path/to/test_file.py                      # Stop on first failure
pytest -n0 path/to/test_file.py                     # Run without parallelization
pytest -m "not slow"                                # Skip slow tests
```

**Test markers:**
- `@pytest.mark.slow` - Long-running tests
- `@pytest.mark.tici` - Tests that only run on device (C3/C3X)
- `@pytest.mark.skip_tici_setup` - Skip device setup fixture

---

## Code Style & Conventions

### General
- **Indentation**: 2 spaces (Python and C++)
- **Line Length**: 160 characters (Python), 120 characters (C++)
- **Priorities**: Safety > Stability > Quality > Features

### Python

**Imports** - Always use absolute imports from the `openpilot` root:
```python
# Correct
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.pid import PIDController

# BANNED - will fail lint
import selfdrive       # Use openpilot.selfdrive
import common          # Use openpilot.common
import system          # Use openpilot.system
from unittest import * # Use pytest instead
```

**Naming:**
- Functions & Variables: `snake_case`
- Classes: `PascalCase`
- Constants: `UPPER_SNAKE_CASE`

**Types:** Use Python 3.11+ type hints. Mypy is configured with strict settings.

**Time:** Always use `time.monotonic()` for intervals/timeouts. `time.time()` is banned.

**Testing:**
- Use `pytest` fixtures and assertions
- `unittest` is banned - use pytest
- Tests auto-run with `OpenpilotPrefix` fixture for clean environment

**Banned APIs (enforced by ruff):**
- `pytest.main` - requires special handling
- `time.time` - use `time.monotonic`
- `pyray.measure_text_ex` - use `openpilot.system.ui.lib.text_measure`
- `pyray.draw_text` - use functions that take font as argument

### C++

**Standard:** C++17 (`-std=c++1z`)

**Naming:** `snake_case` for functions and variables

**Memory:** Use `std::unique_ptr` and `std::shared_ptr`

**Logging** (from `common/swaglog.h`):
```cpp
LOGD("debug: %s", msg);    // Debug
LOG("info: %d", val);      // Info
LOGW("warning: %s", msg);  // Warning
LOGE("error: %s", msg);    // Error

// Rate-limited variants (2 messages per 100ms)
LOGD_100("rate limited debug");
```

**Assertions:** Use `assert()` for invariants that should never be violated.

### Error Handling

**Python:**
- Use descriptive built-in exceptions: `ValueError`, `RuntimeError`, `TypeError`
- Never silently catch and ignore exceptions in safety-critical code

**C++:**
- Return error codes or use assertions for unrecoverable states
- Always check return values from hardware communication

**Hardware:** Always handle potential communication failures with Panda, sensors, cameras.

---

## Repository Structure

```
cereal/          # Cap'n Proto messaging definitions
common/          # Shared library code (params, timing, transformations)
msgq/            # Message queue implementation
opendbc/         # CAN message definitions (DBC files)
panda/           # Panda hardware interface code
rednose/         # Kalman filter library
selfdrive/       # Main driving logic
  car/           # Car-specific interfaces and controllers
  controls/      # Control loops (lateral, longitudinal)
  selfdrived/    # Main state machine and event handling
  locationd/     # Localization (GPS, IMU fusion)
  modeld/        # ML model runners
  ui/            # User interface
system/          # System daemons
  manager/       # Process manager
  loggerd/       # Data logging
  camerad/       # Camera handling
  hardware/      # Hardware abstraction
sunnypilot/      # sunnypilot-specific features
tools/           # Development tools (replay, cabana)
```

---

## Key Patterns

### Messaging (cereal)
Processes communicate via Pub/Sub using cereal/msgq:
```python
from cereal.messaging import SubMaster, PubMaster

# Subscribe to messages
sm = SubMaster(['carState', 'controlsState'])
sm.update()
if sm.updated['carState']:
  speed = sm['carState'].vEgo

# Publish messages
pm = PubMaster(['sendcan'])
pm.send('sendcan', can_msg)
```

### Parameters
Persistent key-value storage for configuration:
```python
from openpilot.common.params import Params

params = Params()
params.get("CarParams")           # Get bytes
params.get("DongleId", encoding='utf-8')  # Get string
params.put("IsOffroad", b"1")     # Put bytes
params.put_bool("IsMetric", True) # Put boolean
params.remove("SomeKey")          # Delete
```

### Car Interface Pattern
Each supported car has three components in `selfdrive/car/<brand>/`:
- `interface.py` - `CarInterface`: Main interface, creates `CarState` and `CarController`
- `carstate.py` - `CarState`: Parses CAN messages into standard format
- `carcontroller.py` - `CarController`: Converts commands to CAN messages

---

## CI/CD Requirements

Before submitting PRs:
1. `scripts/lint/lint.sh` must pass
2. `pytest` must pass for affected code
3. No files larger than 120KB
4. Executable scripts must have correct shebangs
5. No `NOMERGE` comments in code

---

## Safety Notes

- **Never** disable safety checks without thorough review
- **Always** test car-specific changes on actual hardware when possible
- **Verify** CAN message changes against DBC specifications
- **Check** that all actuator commands have appropriate limits
