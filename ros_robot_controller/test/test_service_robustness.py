#!/usr/bin/env python3
"""
Test service robustness with invalid inputs.

Services should handle bad data gracefully, not crash the node.
"""
import pytest
import rclpy
from ros_robot_controller_msgs.srv import GetPWMServoState, GetBusServoState
from ros_robot_controller_msgs.msg import GetPWMServoCmd, GetBusServoCmd


@pytest.fixture(scope='module')
def ros_context():
    """Initialize ROS for testing"""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pwm_servo_service_with_empty_request(ros_context):
    """Service should handle empty request without crashing"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    # Create node (will fail to connect to hardware, but that's OK for testing)
    try:
        node = RosRobotController('test_node')
    except Exception:
        # If hardware connection fails, that's expected in test environment
        pytest.skip("Cannot test without hardware connection")
        return

    request = GetPWMServoState.Request()
    request.cmd = []  # Empty request

    response = GetPWMServoState.Response()

    # Should not raise exception
    try:
        result = node.get_pwm_servo_state(request, response)
        assert result.success == True
        assert len(result.state) == 0
    except Exception as e:
        pytest.fail(f"Service crashed with empty request: {e}")
    finally:
        node.destroy_node()


def test_pwm_servo_service_with_invalid_ids(ros_context):
    """Service should handle invalid servo IDs gracefully"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    try:
        node = RosRobotController('test_node')
    except Exception:
        pytest.skip("Cannot test without hardware connection")
        return

    request = GetPWMServoState.Request()

    # Test with various invalid IDs
    invalid_ids = [-1, 256, 999, -999]

    for invalid_id in invalid_ids:
        cmd = GetPWMServoCmd()
        cmd.id = invalid_id
        cmd.get_position = True
        request.cmd = [cmd]

        response = GetPWMServoState.Response()

        try:
            # Should not crash - may return error or empty result
            result = node.get_pwm_servo_state(request, response)
            # We just verify it doesn't crash
            assert result is not None
        except Exception as e:
            pytest.fail(f"Service crashed with invalid ID {invalid_id}: {e}")

    node.destroy_node()


def test_bus_servo_service_with_invalid_ids(ros_context):
    """Service should handle invalid bus servo IDs gracefully"""
    from ros_robot_controller.ros_robot_controller_node import RosRobotController

    try:
        node = RosRobotController('test_node')
    except Exception:
        pytest.skip("Cannot test without hardware connection")
        return

    request = GetBusServoState.Request()

    # Test with various invalid IDs
    invalid_ids = [-1, 256, 999]

    for invalid_id in invalid_ids:
        cmd = GetBusServoCmd()
        cmd.id = invalid_id
        cmd.get_id = True
        request.cmd = [cmd]

        response = GetBusServoState.Response()

        try:
            result = node.get_bus_servo_state(request, response)
            assert result is not None
        except Exception as e:
            pytest.fail(f"Service crashed with invalid ID {invalid_id}: {e}")

    node.destroy_node()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
