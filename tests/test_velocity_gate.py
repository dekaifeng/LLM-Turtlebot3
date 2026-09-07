import pytest

from turtlebot3_multimodal.executor import Velocity
from turtlebot3_multimodal.guarded_model import write_guarded_model
from turtlebot3_multimodal.velocity_gate import VelocityGate


def test_estop_blocks_continuing_navigation_until_reset_and_fresh_input():
    gate = VelocityGate()
    gate.receive("nav", Velocity(0.1), 0)
    assert gate.tick(0) == Velocity(0.1)
    gate.stop()
    gate.receive("nav", Velocity(0.2), 0.1)
    assert gate.tick(0.1) == Velocity()
    gate.reset()
    assert gate.tick(0.2) == Velocity()
    gate.receive("nav", Velocity(0.1), 0.3)
    assert gate.tick(0.3) == Velocity(0.1)


def test_exclusive_source_selection_drops_old_commands_and_limits_speed():
    gate = VelocityGate()
    gate.receive("manual", Velocity(-0.1), 0)
    assert gate.tick(0) == Velocity()
    gate.receive("nav", Velocity(9, 9), 0.1)
    assert gate.tick(0.1) == Velocity(0.22, 1.5)
    gate.select("manual")
    assert gate.tick(0.2) == Velocity()
    gate.receive("nav", Velocity(0.2), 0.2)
    gate.receive("manual", Velocity(-0.1), 0.2)
    assert gate.tick(0.2) == Velocity(-0.1)


def test_input_watchdog_latches_stop_even_when_output_timer_is_alive():
    gate = VelocityGate()
    gate.receive("nav", Velocity(0.1), 0)
    for tick in (0, 0.2, 0.4):
        assert gate.tick(tick) == Velocity(0.1)
    assert gate.tick(0.6) == Velocity()
    gate.receive("nav", Velocity(0.1), 0.7)
    assert gate.tick(0.7) == Velocity()


@pytest.mark.parametrize("time", [float("nan"), -1, 1])
def test_invalid_or_discontinuous_output_clock_stops(time):
    gate = VelocityGate()
    gate.receive("nav", Velocity(0.1), 0)
    gate.tick(0)
    assert gate.tick(time) == Velocity()
    assert gate.stopped


def test_model_routing_fails_closed_for_unexpected_models(tmp_path):
    source, target = tmp_path / "in.sdf", tmp_path / "out.sdf"
    source.write_text('<sdf><plugin filename="libgazebo_ros_diff_drive.so">'
                      '<command_topic>cmd_vel</command_topic></plugin></sdf>')
    write_guarded_model(str(source), str(target))
    assert "/cmd_vel_safe" in target.read_text()
    source.write_text("<sdf/>")
    with pytest.raises(ValueError, match="exactly one"):
        write_guarded_model(str(source), str(target))
