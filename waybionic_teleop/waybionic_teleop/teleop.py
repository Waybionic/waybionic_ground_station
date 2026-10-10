"""Turn Xbox controller input into arm joint targets (pure logic, no ROS)."""

from dataclasses import dataclass
import math

from waybionic_teleop.gamepad import AXIS, BUTTON

ACTIONS = ('enable', 'stop', 'group', 'home', 'faster', 'slower', 'lock', 'tilt_up', 'tilt_down')
CARTESIAN_AXES = ('x', 'y', 'z', 'roll')


def clamp(value, low, high):
    return min(max(value, low), high)


@dataclass
class Group:
    """Joints that share the sticks; each joint follows one axis with a signed gain."""

    name: str
    joints: list
    axes: list
    scales: list
    # 'cartesian' groups move the tool tip along x, y, z and roll instead of single joints.
    mode: str = 'joint'

    def describe(self):
        moves = CARTESIAN_AXES if self.mode == 'cartesian' else self.joints
        return ', '.join(f'{axis} {move}' for axis, move in zip(self.axes, moves))


@dataclass
class TeleopConfig:
    """Controller mapping and motion limits from config/xbox_teleop.yaml."""

    groups: list
    buttons: dict
    tool_joint: str
    tool_limits: tuple
    tool_speed: float
    tool_close_axis: str
    tool_open_axis: str
    period: float
    max_speed: float
    max_accel: float
    linear_speed: float
    linear_accel: float
    tilt_speed: float
    speed_levels: list
    speed_level: int
    deadzone: float
    home_gain: float


def config_from_parameters(params):
    """Build a TeleopConfig from flat ROS parameter names such as 'base.joints'."""
    try:
        rate_hz = float(params['rate_hz'])
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('rate_hz must be positive')
        groups = [Group(name, list(params[f'{name}.joints']), list(params[f'{name}.axes']),
                        [float(scale) for scale in params[f'{name}.scales']],
                        params.get(f'{name}.mode', 'joint'))
                  for name in params['groups']]
        config = TeleopConfig(
            groups=groups,
            buttons={action: params[f'{action}_button'] for action in ACTIONS},
            tool_joint=params['tool_joint'],
            tool_limits=tuple(float(value) for value in params['tool_limits']),
            tool_speed=float(params['tool_speed']),
            tool_close_axis=params['tool_close_axis'],
            tool_open_axis=params['tool_open_axis'],
            period=1.0 / rate_hz,
            max_speed=math.radians(params['max_speed_deg_s']),
            max_accel=math.radians(params['max_accel_deg_s2']),
            linear_speed=float(params['max_linear_speed_mm_s']) / 1000.0,
            linear_accel=float(params['max_linear_accel_mm_s2']) / 1000.0,
            tilt_speed=math.radians(params['max_tilt_speed_deg_s']),
            speed_levels=[float(level) for level in params['speed_levels']],
            speed_level=int(params['initial_speed_level']),
            deadzone=float(params['deadzone']),
            home_gain=float(params['home_gain']))
    except KeyError as missing:
        raise ValueError(f'missing teleop parameter {missing}') from None
    if not math.isfinite(config.period):
        raise ValueError('rate_hz is too small for a finite control period')
    if not config.groups:
        raise ValueError('at least one teleop group is required')
    speeds = (config.tool_speed, config.max_speed, config.max_accel, config.linear_speed,
              config.linear_accel, config.tilt_speed, config.home_gain)
    if any(not math.isfinite(value) or value <= 0 for value in speeds):
        raise ValueError('teleop speeds and accelerations must be positive and finite')
    if any(not math.isfinite(value) or not 0 < value <= 1 for value in config.speed_levels):
        raise ValueError('speed_levels must be in (0, 1]')
    for group in config.groups:
        if any(not math.isfinite(scale) for scale in group.scales):
            raise ValueError(f'group {group.name} scales must be finite')
        if group.mode == 'cartesian':
            if not len(group.axes) == len(group.scales) == len(CARTESIAN_AXES):
                raise ValueError(f'group {group.name} needs x, y, z and roll axes and scales')
        elif group.mode != 'joint':
            raise ValueError(f'group {group.name} mode must be joint or cartesian')
        elif not len(group.joints) == len(group.axes) == len(group.scales):
            raise ValueError(f'group {group.name} needs one axis and scale per joint')
    unknown = [name for name in [axis for group in config.groups for axis in group.axes]
               + [config.tool_close_axis, config.tool_open_axis] if name not in AXIS]
    unknown += [name for name in config.buttons.values() if name not in BUTTON]
    if unknown:
        raise ValueError('unknown controller inputs: ' + ', '.join(map(str, unknown)))
    if not 0 <= config.speed_level < len(config.speed_levels) or not 0 <= config.deadzone < 1:
        raise ValueError('initial_speed_level or deadzone out of range')
    if len(config.tool_limits) != 2 or not config.tool_limits[0] < config.tool_limits[1]:
        raise ValueError('tool_limits must be [open, closed] with open < closed')
    return config


class ArmTeleop:
    """Hold joint targets and move them with the active group's sticks while enabled."""

    def __init__(self, config, limits, kinematics=None):
        """Take the config, {joint: (lower, upper)} radians and optional ArmKinematics."""
        self.config = config
        self.kinematics = kinematics
        # Without kinematics for this arm, the Cartesian groups are left out.
        self.groups = [group for group in config.groups
                       if group.mode != 'cartesian' or kinematics is not None]
        if not self.groups:
            raise ValueError('no joint-space group or compatible Cartesian kinematics')
        for group in self.groups:
            if group.mode == 'cartesian' and (
                    tuple(group.joints) != kinematics.joints
                    or any(joint not in limits for joint in kinematics.joints)):
                raise ValueError(f'Cartesian group {group.name} does not match the URDF chain')
        self.limits = dict(limits)
        self.limits[config.tool_joint] = config.tool_limits
        self.enabled = False
        self.group = 0
        self.level = config.speed_level
        self.targets = {}
        # Published positions are the next-period setpoints, not joint-wise extrapolations
        # of a Cartesian solve that could leave the straight line at a limit.
        self.command_targets = {}
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.linear = (0.0, 0.0, 0.0)
        self.tilt = 0.0
        self.roll = 0.0
        self.held = set()
        self.wait_for_center = False
        self.blocked = []
        self.note = 'Press Start (Xbox Menu button) to enable'
        self.warning = False

    @property
    def active_group(self):
        return self.groups[self.group]

    @property
    def speed(self):
        return self.config.max_speed * self.config.speed_levels[self.level]

    @property
    def linear_speed(self):
        return self.config.linear_speed * self.config.speed_levels[self.level]

    def update(self, axes, buttons, measured, dt):
        """Apply one controller sample; return True when the targets should be sent."""
        held = {name for name, index in BUTTON.items() if index < len(buttons) and buttons[index]}
        pressed = {action for action, name in self.config.buttons.items()
                   if name in held - self.held}
        self.held = held
        if 'stop' in pressed or ('enable' in pressed and self.enabled):
            self.disable(measured, 'Stopped; press Start (Xbox Menu button) to enable')
            return True
        if 'enable' in pressed:
            self.enable(measured, axes)
        if 'group' in pressed:
            self.group = (self.group + 1) % len(self.groups)
            self.stop_cartesian()
            self.velocities = dict.fromkeys(self.limits, 0.0)
            self.command_targets = dict(self.targets)
            self.wait_for_center = True
        if 'faster' in pressed:
            self.level = min(self.level + 1, len(self.config.speed_levels) - 1)
        if 'slower' in pressed:
            self.level = max(self.level - 1, 0)
        if not self.enabled:
            return False
        if self.wait_for_center:
            if self.motion_input_held(axes):
                self.note = 'Center motion controls before moving in the new group'
                self.warning = True
                return True
            self.wait_for_center = False
            self.note, self.warning = '', False
        self.move(axes, self.config.buttons['home'] in held, dt)
        return True

    def motion_input_held(self, axes):
        config = self.config
        sticks = {axis for group in config.groups for axis in group.axes}
        buttons = {config.buttons[action] for action in ('home', 'tilt_up', 'tilt_down')}
        return (any(self.stick(axes, axis) for axis in sticks)
                or any(self.trigger(axes, axis) > config.deadzone
                       for axis in (config.tool_close_axis, config.tool_open_axis))
                or bool(buttons & self.held))

    def enable(self, measured, axes):
        missing = [joint for joint in self.limits
                   if joint not in measured or not math.isfinite(measured[joint])]
        if missing:
            self.note, self.warning = 'Waiting for joint states: ' + ', '.join(missing), True
            return
        if self.motion_input_held(axes):
            # A stuck or held input must never start moving the arm the moment it is enabled.
            self.note = 'Center the sticks and release the triggers and buttons, then press Start'
            self.warning = True
            return
        # Start from the measured pose so enabling never makes the arm jump.
        self.targets = {joint: measured[joint] for joint in self.limits}
        self.command_targets = dict(self.targets)
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.stop_cartesian()
        self.wait_for_center = False
        self.enabled, self.note, self.warning = True, '', False

    def disable(self, measured, note, warning=False):
        self.enabled, self.note, self.warning = False, note, warning
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.stop_cartesian()
        self.wait_for_center = False
        self.targets.update({joint: measured[joint] for joint in self.limits
                             if joint in measured and math.isfinite(measured[joint])})
        self.command_targets = dict(self.targets)

    def stop_cartesian(self):
        self.linear, self.tilt, self.roll = (0.0, 0.0, 0.0), 0.0, 0.0

    def move(self, axes, homing, dt):
        config = self.config
        self.command_targets = dict(self.targets)
        desired = dict.fromkeys(self.limits, 0.0)
        self.blocked = []
        if homing:
            self.stop_cartesian()
            for joint, (lower, upper) in self.limits.items():
                if joint != config.tool_joint:
                    error = clamp(0.0, lower, upper) - self.targets[joint]
                    desired[joint] = clamp(config.home_gain * error, -self.speed, self.speed)
        elif self.active_group.mode == 'cartesian':
            self.jog(axes, dt)
            desired = {config.tool_joint: 0.0}
        else:
            group = self.active_group
            for joint, axis, scale in zip(group.joints, group.axes, group.scales):
                desired[joint] += scale * self.stick(axes, axis) * self.speed
        desired[config.tool_joint] = config.tool_speed * (
            self.trigger(axes, config.tool_close_axis) - self.trigger(axes, config.tool_open_axis))
        step = config.max_accel * dt
        for joint, goal in desired.items():
            velocity = self.velocities[joint]
            if joint == config.tool_joint:
                velocity = goal
            else:
                velocity += clamp(goal - velocity, -step, step)
            target = self.targets[joint] + velocity * dt
            lower, upper = self.limits[joint]
            # Stop at a limit, but never pull back a target that started outside it.
            if velocity > 0 and target > upper:
                target, velocity = max(self.targets[joint], upper), 0.0
                self.blocked.append(joint)
            elif velocity < 0 and target < lower:
                target, velocity = min(self.targets[joint], lower), 0.0
                self.blocked.append(joint)
            self.targets[joint], self.velocities[joint] = target, velocity
            self.command_targets[joint] = clamp(
                target + velocity * config.period, min(lower, target), max(upper, target))

    def jog(self, axes, dt):
        """Move the tool tip along a straight line set by the sticks, or tilt the tool about it."""
        config, group = self.config, self.active_group
        values = [scale * self.stick(axes, axis) for axis, scale in zip(group.axes, group.scales)]
        linear, roll = values[:3], values[3]
        if config.buttons['lock'] in self.held:
            dominant = max(range(3), key=lambda index: abs(linear[index]))
            linear = [value if index == dominant else 0.0 for index, value in enumerate(linear)]
        length = math.hypot(*linear)
        if length > 1.0:
            # A diagonal must not move the tip faster than the top speed along one axis.
            linear = [value / length for value in linear]
        # Ramp the tip velocity as one vector, so speeding up or slowing down never bends the line.
        change = [self.linear_speed * goal - current for goal, current in zip(linear, self.linear)]
        size, most = math.sqrt(sum(value * value for value in change)), config.linear_accel * dt
        if size > most:
            change = [value * most / size for value in change]
        velocity = [current + value for current, value in zip(self.linear, change)]
        # Tilting up turns the tool axis towards straight up, which lowers the pitch angle.
        tilt = config.tilt_speed * config.speed_levels[self.level] * (
            (config.buttons['tilt_down'] in self.held) - (config.buttons['tilt_up'] in self.held))
        step = config.max_accel * dt
        tilt = self.tilt + clamp(tilt - self.tilt, -step, step)
        roll = self.roll + clamp(roll * self.speed - self.roll, -step, step)
        before = {joint: self.targets[joint] for joint in self.kinematics.joints}
        after, fraction, self.blocked = self.kinematics.jog(
            before, velocity, tilt, roll, dt, self.limits, config.max_speed)
        self.linear = tuple(value * fraction for value in velocity)
        self.tilt, self.roll = tilt * fraction, roll * fraction
        for joint in self.kinematics.joints:
            self.velocities[joint] = (after[joint] - before[joint]) / dt
            self.targets[joint] = after[joint]
        if any(self.linear) or self.tilt or self.roll:
            future, _, _ = self.kinematics.jog(
                after, self.linear, self.tilt, self.roll, config.period, self.limits,
                config.max_speed)
        else:
            future = after
        self.command_targets.update(future)

    def stick(self, axes, name):
        value = self.axis(axes, name)
        magnitude = abs(value) - self.config.deadzone
        if magnitude <= 0:
            return 0.0
        return math.copysign(min(magnitude / (1.0 - self.config.deadzone), 1.0), value)

    def trigger(self, axes, name):
        # game_controller_node triggers rest at 0 and reach -1 when fully pressed.
        return clamp(-self.axis(axes, name), 0.0, 1.0)

    @staticmethod
    def axis(axes, name):
        index = AXIS[name]
        value = float(axes[index]) if index < len(axes) else 0.0
        return value if math.isfinite(value) else 0.0
