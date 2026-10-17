"""The tool board's motor logic, run on a computer with a fake stepper driver."""

from pathlib import Path
import shutil
import subprocess

import pytest

from waybionic_teleop import mks_can

FIRMWARE = Path(__file__).resolve().parents[2] / 'firmware' / 'tool_board'
ID = 7
TURN = mks_can.COUNTS_PER_REV

# Drives one ToolMotor (3200 steps per turn, one turn of travel either way) from stdin:
#   frame HEX    hand it a frame        wait MS   let time pass, then update once
#   run N        update up to N times   state     print the fake stepper
#   coast N      let the stepper run N steps past its target, as AccelStepper's ramp can
# Each command prints the reply in hex, or - for none.
HARNESS = r"""
#include <cstdio>
#include <cstring>
#include "tool_motor.h"

struct FakeStepper
{
  long position = 0;
  long target = 0;
  float max_speed = 0.0f;
  float acceleration = 0.0f;
  bool braking = false;
  long coast = 0;
  int heading = 0;
  void moveTo(long goal) {target = goal;}
  // Runs at full speed until it reaches the target, and stops like AccelStepper::stop().
  float speed() {return (coasting() ? heading : toward()) * max_speed;}
  void stop()
  {
    braking = true;
    const float velocity = speed();
    if (velocity != 0.0f) {
      const long steps = static_cast<long>(velocity * velocity / (2.0 * acceleration)) + 1;
      target = position + (velocity > 0 ? steps : -steps);
    }
  }
  bool run()
  {
    if (coasting()) {
      position += heading;
      --coast;
    } else if (toward()) {
      heading = toward();
      position += heading;
    }
    return position != target;
  }
  void setCurrentPosition(long value) {position = target = value; coast = 0;}
  long currentPosition() {return position;}
  long targetPosition() {return target;}
  long distanceToGo() {return target - position;}
  void setMaxSpeed(float value) {max_speed = value;}
  void setAcceleration(float value) {acceleration = value;}
  int toward() const {return position < target ? 1 : position > target ? -1 : 0;}
  bool coasting() const {return coast && heading && toward() != heading;}
};

static void print(const uint8_t * data, uint8_t length)
{
  if (!length) {
    std::printf("-\n");
    return;
  }
  for (uint8_t i = 0; i < length; ++i) {
    std::printf("%02X", data[i]);
  }
  std::printf("\n");
}

int main()
{
  FakeStepper stepper;
  ToolMotor<FakeStepper> motor(7, stepper, MotorConfig{3200, -3200, 3200, 3200.0f, 20000.0f});
  uint32_t now = 0;
  char command[16];
  while (std::scanf("%15s", command) == 1) {
    uint8_t out[8];
    uint8_t length = 0;
    if (std::strcmp(command, "frame") == 0) {
      char hex[32];
      uint8_t data[8];
      if (std::scanf("%31s", hex) != 1) {
        return 1;
      }
      const size_t size = std::strlen(hex) / 2;
      for (size_t i = 0; i < size && i < 8; ++i) {
        if (std::sscanf(hex + 2 * i, "%2hhx", &data[i]) != 1) {
          return 1;
        }
      }
      print(out, motor.handle(data, static_cast<uint8_t>(size), now, out));
    } else if (std::strcmp(command, "wait") == 0) {
      unsigned milliseconds;
      if (std::scanf("%u", &milliseconds) != 1) {
        return 1;
      }
      now += milliseconds;
      print(out, motor.update(now, out));
    } else if (std::strcmp(command, "run") == 0) {
      int steps;
      if (std::scanf("%d", &steps) != 1) {
        return 1;
      }
      for (int i = 0; i < steps && !length; ++i) {
        length = motor.update(now, out);
      }
      print(out, length);
    } else if (std::strcmp(command, "coast") == 0) {
      if (std::scanf("%ld", &stepper.coast) != 1) {
        return 1;
      }
      std::printf("-\n");
    } else {
      std::printf("%ld %ld %.0f %.0f %d %d\n", stepper.position, stepper.target,
                  stepper.max_speed, stepper.acceleration, stepper.braking, motor.enabled());
    }
  }
  return 0;
}
"""


@pytest.fixture(scope='module')
def program(tmp_path_factory):
    compiler = shutil.which('g++') or shutil.which('c++')
    if compiler is None:
        pytest.skip('needs a C++ compiler')
    directory = tmp_path_factory.mktemp('tool_motor')
    (directory / 'harness.cpp').write_text(HARNESS, encoding='utf-8')
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(FIRMWARE),
                    str(directory / 'harness.cpp'), '-o', str(directory / 'harness')],
                   check=True)
    return directory / 'harness'


def motor(program, *commands):
    """Run commands against a fresh motor; return one output line per command."""
    lines = [f'frame {command.hex()}' if isinstance(command, bytes) else command
             for command in commands]
    result = subprocess.run([str(program)], input='\n'.join(lines) + '\n', capture_output=True,
                            text=True, check=True)
    return result.stdout.splitlines()


def status(code, value):
    return mks_can.frame(ID, code, [value]).hex().upper()


def state(line):
    position, target, speed, acceleration, braking, enabled = line.split()
    return {'position': int(position), 'target': int(target), 'speed': float(speed),
            'acceleration': float(acceleration), 'braking': braking == '1',
            'enabled': enabled == '1'}


READY = (mks_can.set_mode(ID), mks_can.enable(ID))


def test_moves_need_bus_mode_and_enable(program):
    out = motor(program, mks_can.absolute_axis(ID, TURN // 2, 30, 200), *READY,
                mks_can.absolute_axis(ID, TURN // 2, 30, 200))
    assert out == [status(mks_can.ABSOLUTE_AXIS, 0), status(mks_can.SET_MODE, 1),
                   status(mks_can.ENABLE, 1), status(mks_can.ABSOLUTE_AXIS, 1)]


def test_a_move_runs_to_its_target_and_reports_it(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN // 2, 30, 200), 'state',
                'run 5000', mks_can.read_encoder(ID))
    moving = state(out[3])
    # Half a turn is 1600 steps, and 30 rpm is 1600 steps per second.
    assert (moving['target'], moving['speed']) == (1600, 1600.0)
    assert out[4] == status(mks_can.ABSOLUTE_AXIS, 2)
    _, arguments = mks_can.parse(ID, bytes.fromhex(out[5]))
    assert mks_can.encoder_value(arguments) == TURN // 2


def test_a_target_past_the_travel_limit_stops_at_it_and_says_so(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, 2 * TURN, 60, 200), 'state',
                'run 5000')
    assert state(out[3])['target'] == 3200
    assert out[4] == status(mks_can.ABSOLUTE_AXIS, 3)


def test_the_heartbeat_stops_a_moving_motor_when_the_host_goes_quiet(program):
    out = motor(program, *READY, mks_can.set_heartbeat(ID, 100),
                mks_can.absolute_axis(ID, TURN, 60, 200), 'run 10', 'wait 50', 'state',
                'wait 60', 'state', 'run 100')
    assert state(out[6])['target'] == 3200
    stopped = state(out[8])
    assert stopped['position'] == stopped['target'] < 100
    assert out[9] == '-'


def test_stop_with_acceleration_zero_is_immediate_and_otherwise_brakes(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 200), 'run 10',
                mks_can.stop(ID, 0), 'state')
    assert out[4] == status(mks_can.ABSOLUTE_AXIS, 1)
    stopped = state(out[5])
    assert stopped['position'] == stopped['target'] == 10 and not stopped['braking']
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 200), 'run 10',
                mks_can.stop(ID, 255), 'state')
    braking = state(out[5])
    assert braking['braking'] and braking['acceleration'] == 20000.0


def test_a_stop_brakes_harder_rather_than_pass_the_travel_limit(program):
    # 400 steps from the limit at 3200 steps/s; acc 1 (4183 steps/s^2) would need 1224 steps.
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 255), 'run 2800',
                mks_can.stop(ID, 1), 'state')
    braking = state(out[5])
    assert (braking['target'], braking['acceleration']) == (3200, 12800.0)
    # 100 steps from it, even the maximum of 20000 steps/s^2 needs 256: stop at once.
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 255), 'run 3100',
                mks_can.stop(ID, 1), 'state')
    held = state(out[5])
    assert held['position'] == held['target'] == 3100


def test_a_gentler_move_brakes_harder_rather_than_pass_the_travel_limit(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 255), 'run 2800',
                mks_can.absolute_axis(ID, 0, 60, 1), 'state')
    turning = state(out[5])
    assert (turning['target'], turning['acceleration']) == (0, 12800.0)
    # With room to stop, the frame's own acceleration stands.
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 255), 'run 1000',
                mks_can.absolute_axis(ID, 0, 60, 1), 'state')
    assert state(out[5])['acceleration'] == 4183.0


def test_a_ramp_that_would_run_on_past_the_travel_limit_stops_at_it(program):
    out = motor(program, *READY, 'coast 3', mks_can.absolute_axis(ID, TURN, 60, 200),
                'run 5000', 'run 3', 'state')
    assert out[4] == status(mks_can.ABSOLUTE_AXIS, 2)
    stopped = state(out[6])
    assert stopped['position'] == stopped['target'] == 3200


def test_the_emergency_stop_holds_at_once_and_says_so(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 200), 'run 10',
                mks_can.emergency_stop(ID), 'state', 'run 100')
    assert out[4] == status(mks_can.EMERGENCY_STOP, 1)
    held = state(out[5])
    assert held['position'] == held['target'] == 10 and not held['braking']
    assert out[6] == '-'


def test_disabling_holds_and_zeroing_makes_here_zero(program):
    out = motor(program, *READY, mks_can.absolute_axis(ID, TURN, 60, 200), 'run 10',
                mks_can.enable(ID, False), 'state', mks_can.absolute_axis(ID, TURN, 60, 200),
                mks_can.set_zero(ID), mks_can.read_encoder(ID))
    held = state(out[5])
    assert held['position'] == held['target'] == 10 and not held['enabled']
    assert out[6] == status(mks_can.ABSOLUTE_AXIS, 0)
    assert out[7] == status(mks_can.SET_ZERO, 1)
    _, arguments = mks_can.parse(ID, bytes.fromhex(out[8]))
    assert mks_can.encoder_value(arguments) == 0
