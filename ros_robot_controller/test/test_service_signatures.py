#!/usr/bin/env python3
"""
Test service callback signatures to catch common ROS2 bugs.

This would have caught the get_pwm_servo_state() signature bug immediately!
"""
import inspect
import pytest


def test_get_pwm_servo_state_signature():
    """Ensure get_pwm_servo_state has correct ROS2 service callback signature"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    sig = inspect.signature(RosRobotController.get_pwm_servo_state)
    params = list(sig.parameters.keys())

    # ROS2 service callbacks must be: (self, request, response)
    assert len(params) == 3, f"Service callback should have 3 params, got {len(params)}: {params}"
    assert params[0] == 'self', "First param should be 'self'"
    assert params[1] == 'request', "Second param should be 'request'"
    assert params[2] == 'response', "Third param should be 'response'"


def test_get_bus_servo_state_signature():
    """Ensure get_bus_servo_state has correct ROS2 service callback signature"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    sig = inspect.signature(RosRobotController.get_bus_servo_state)
    params = list(sig.parameters.keys())

    assert len(params) == 3, f"Service callback should have 3 params, got {len(params)}: {params}"
    assert params[0] == 'self'
    assert params[1] == 'request'
    assert params[2] == 'response'


def test_all_service_signatures():
    """Check all service callbacks have correct signatures"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    # List all methods that might be service callbacks
    service_methods = [
        name for name in dir(RosRobotController)
        if not name.startswith('_') and callable(getattr(RosRobotController, name))
    ]

    # Check known service callbacks
    known_services = ['get_pwm_servo_state', 'get_bus_servo_state']

    for service_name in known_services:
        if service_name in service_methods:
            method = getattr(RosRobotController, service_name)
            sig = inspect.signature(method)
            params = list(sig.parameters.keys())

            assert len(params) == 3, \
                f"{service_name} should have 3 params (self, request, response), got {params}"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
