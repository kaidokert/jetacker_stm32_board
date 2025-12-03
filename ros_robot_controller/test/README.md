# ROS Robot Controller Tests

Regression tests to catch common bugs and ensure robustness.

## What These Tests Catch

### test_service_signatures.py
**Catches:** Wrong service callback signatures (the bug we just fixed!)

The `get_pwm_servo_state()` bug would have been caught immediately:
```
TypeError: RosRobotController.get_pwm_servo_state() takes 2 positional arguments but 3 were given
```

This test verifies all service callbacks have the correct signature: `(self, request, response)`

### test_imports.py
**Catches:** Missing imports (also caught our bug!)

The missing `PWMServoState` import would have been detected before runtime:
```
NameError: name 'PWMServoState' is not defined
```

### test_service_robustness.py
**Catches:** Crashes from invalid input

Tests that services handle:
- Empty requests
- Invalid servo IDs (negative, >255, etc.)
- Malformed data

Services should log errors or return failure, NOT crash the node.

## Running Tests

### Quick Test (No Hardware)
```bash
# Run static tests only (signatures, imports)
cd /workspace/src/ros_robot_controller
python3 -m pytest test/test_service_signatures.py -v
python3 -m pytest test/test_imports.py -v
```

### Full Test Suite
```bash
# Build workspace first
cd /workspace
colcon build --packages-select ros_robot_controller

# Run all tests
colcon test --packages-select ros_robot_controller

# View results
colcon test-result --verbose
```

### Run Specific Test
```bash
python3 -m pytest test/test_service_signatures.py::test_get_pwm_servo_state_signature -v
```

## Expected Results

**Without hardware connected:**
- ✅ `test_service_signatures.py` - Should PASS
- ✅ `test_imports.py` - Should PASS
- ⏭️ `test_service_robustness.py` - Will SKIP (needs hardware)

**With hardware connected:**
- ✅ All tests should PASS

## CI/CD Integration

Add to GitHub Actions:
```yaml
- name: Run ROS tests
  run: |
    source /opt/ros/jazzy/setup.bash
    colcon test --packages-select ros_robot_controller
    colcon test-result --verbose
```

## What To Do When Tests Fail

1. **Signature mismatch** → Fix service callback to use `(self, request, response)`
2. **Import error** → Add missing import to ros_robot_controller_node.py
3. **Robustness failure** → Add try/except and input validation

## Future Improvements

- [ ] Add property-based testing with Hypothesis
- [ ] Add launch tests for integration testing
- [ ] Add mock hardware for testing without real board
- [ ] Add performance tests (message rate, latency)
- [ ] Add coverage reporting

## Why These Tests Matter

**Before tests:** Node crashes when bad service call is made, robot stops working

**After tests:** Bug caught during development, never reaches production

These simple tests would have prevented the production crash we just debugged!
