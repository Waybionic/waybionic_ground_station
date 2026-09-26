"""Turn Xbox controller input into arm joint targets (pure logic, no ROS)."""

from dataclasses import dataclass
import math

from waybionic_teleop.gamepad import AXIS, BUTTON

ACTIONS = ('enable', 'stop', 'group', 'home', 'faster', 'slower', 'lock')
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
    max_speed: float
    max_accel: float
    linear_speed: float
    linear_accel: float
    speed_levels: list
    speed_level: int
    deadzone: float
    home_gain: float


def config_from_parameters(params):
    """Build a TeleopConfig from flat ROS parameter names such as 'base.joints'."""
    try:
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
            max_speed=math.radians(params['max_speed_deg_s']),
            max_accel=math.radians(params['max_accel_deg_s2']),
            linear_speed=params['max_linear_speed_mm_s'] / 1000.0,
            linear_accel=params['max_linear_accel_mm_s2'] / 1000.0,
            speed_levels=[float(level) for level in params['speed_levels']],
            speed_level=int(params['initial_speed_level']),
            deadzone=float(params['deadzone']),
            home_gain=float(params['home_gain']))
    except KeyError as missing:
        raise ValueError(f'missing teleop parameter {missing}') from None
    for group in config.groups:
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
    if not config.tool_limits[0] < config.tool_limits[1]:
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
        self.limits = dict(limits)
        self.limits[config.tool_joint] = config.tool_limits
        self.enabled = False
        self.group = 0
        self.level = config.speed_level
        self.targets = {}
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.linear = (0.0, 0.0, 0.0)
        self.held = set()
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
            self.linear = (0.0, 0.0, 0.0)
        if 'faster' in pressed:
            self.level = min(self.level + 1, len(self.config.speed_levels) - 1)
        if 'slower' in pressed:
            self.level = max(self.level - 1, 0)
        if not self.enabled:
            return False
        self.move(axes, self.config.buttons['home'] in held, dt)
        return True

    def enable(self, measured, axes):
        missing = [joint for joint in self.limits if joint not in measured]
        if missing:
            self.note, self.warning = 'Waiting for joint states: ' + ', '.join(missing), True
            return
        config = self.config
        sticks = {axis for group in config.groups for axis in group.axes}
        if (any(self.stick(axes, axis) for axis in sticks)
                or any(self.trigger(axes, axis) > config.deadzone
                       for axis in (config.tool_close_axis, config.tool_open_axis))):
            # A stuck or held input must never start moving the arm the moment it is enabled.
            self.note = 'Center the sticks and release the triggers, then press Start'
            self.warning = True
            return
        # Start from the measured pose so enabling never makes the arm jump.
        self.targets = {joint: measured[joint] for joint in self.limits}
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.linear = (0.0, 0.0, 0.0)
        self.enabled, self.note, self.warning = True, '', False

    def disable(self, measured, note, warning=False):
        self.enabled, self.note, self.warning = False, note, warning
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.linear = (0.0, 0.0, 0.0)
        self.targets.update({joint: measured[joint] for joint in self.limits if joint in measured})

    def move(self, axes, homing, dt):
        config = self.config
        desired = dict.fromkeys(self.limits, 0.0)
        self.blocked = []
        if homing:
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

    def jog(self, axes, dt):
        """Move the tool tip along a straight line set by the sticks, keeping the tool pitch."""
        config, group = self.config, self.active_group
        values = [scale * self.stick(axes, axis) for axis, scale in zip(group.axes, group.scales)]
        linear, roll = values[:3], values[3]
        if config.buttons['lock'] in self.held:
            dominant = max(range(3), key=lambda index: abs(linear[index]))
            linear = [value if index == dominant else 0.0 for index, value in enumerate(linear)]
        # Ramp the tip velocity as one vector, so speeding up or slowing down never bends the line.
        change = [self.linear_speed * goal - current for goal, current in zip(linear, self.linear)]
        size, most = math.sqrt(sum(value * value for value in change)), config.linear_accel * dt
        if size > most:
            change = [value * most / size for value in change]
        velocity = [current + value for current, value in zip(self.linear, change)]
        before = {joint: self.targets[joint] for joint in self.kinematics.joints}
        after, fraction, self.blocked = self.kinematics.jog(
            before, velocity, roll * self.speed, dt, self.limits, config.max_speed)
        self.linear = tuple(value * fraction for value in velocity)
        for joint in self.kinematics.joints:
            self.velocities[joint] = (after[joint] - before[joint]) / dt
            self.targets[joint] = after[joint]

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
