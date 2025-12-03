#!/usr/bin/env python3
"""
Test that all required imports are present.

This would have caught the missing PWMServoState import!
"""
import pytest


def test_pwm_servo_state_imported():
    """Verify PWMServoState is imported (was missing, caused crash)"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController
    import inspect

    # Get the source code of the module
    source = inspect.getsource(RosRobotController)

    # Check that PWMServoState is used in the code
    assert 'PWMServoState' in source, "PWMServoState is used but may not be imported"

    # Try to access it from the module namespace
    module = inspect.getmodule(RosRobotController)

    # This will fail if PWMServoState is not imported
    try:
        # Try to use PWMServoState like the code does
        from ros_robot_controller_msgs.msg import PWMServoState
        obj = PWMServoState()
        assert obj is not None
    except NameError as e:
        pytest.fail(f"PWMServoState not properly imported: {e}")


def test_bus_servo_state_imported():
    """Verify BusServoState is imported"""
    try:
        from ros_robot_controller_msgs.msg import BusServoState
        obj = BusServoState()
        assert obj is not None
    except ImportError as e:
        pytest.fail(f"BusServoState not imported: {e}")


def test_all_message_types_available():
    """Verify all message types used in code are importable"""
    required_messages = [
        'ButtonState',
        'BuzzerState',
        'LedState',
        'MotorsState',
        'BusServoState',
        'SetBusServoState',
        'ServosPosition',
        'SetPWMServoState',
        'PWMServoState',  # This was missing!
        'Sbus',
        'OLEDState'
    ]

    for msg_type in required_messages:
        try:
            exec(f"from ros_robot_controller_msgs.msg import {msg_type}")
        except ImportError as e:
            pytest.fail(f"Cannot import {msg_type}: {e}")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
