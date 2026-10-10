"""Turn Xbox controller input into arm joint targets (pure logic, no ROS)."""

from dataclasses import dataclass
import math

from waybionic_teleop.gamepad import AXIS, BUTTON

ACTIONS = ('enable', 'stop', 'group', 'home', 'faster', 'slower')


def clamp(value, low, high):
    return min(max(value, low), high)


@dataclass
class Group:
    """Joints that share the sticks; each joint follows one axis with a signed gain."""

    name: str
    joints: list
    axes: list
    scales: list

    def describe(self):
        return ', '.join(f'{axis} {joint}' for axis, joint in zip(self.axes, self.joints))


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
    speed_levels: list
    speed_level: int
    deadzone: float
    home_gain: float


def config_from_parameters(params):
    """Build a TeleopConfig from flat ROS parameter names such as 'base.joints'."""
    try:
        groups = [Group(name, list(params[f'{name}.joints']), list(params[f'{name}.axes']),
                        [float(scale) for scale in params[f'{name}.scales']])
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
            speed_levels=[float(level) for level in params['speed_levels']],
            speed_level=int(params['initial_speed_level']),
            deadzone=float(params['deadzone']),
            home_gain=float(params['home_gain']))
    except KeyError as missing:
        raise ValueError(f'missing teleop parameter {missing}') from None
    for group in config.groups:
        if not len(group.joints) == len(group.axes) == len(group.scales):
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

    def __init__(self, config, limits):
        """Take the config and {joint: (lower, upper)} radians for every grouped joint."""
        self.config = config
        self.limits = dict(limits)
        self.limits[config.tool_joint] = config.tool_limits
        self.enabled = False
        self.group = 0
        self.level = config.speed_level
        self.targets = {}
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.held = set()
        self.blocked = []
        self.note = 'Press Start (Xbox Menu button) to enable'
        self.warning = False

    @property
    def active_group(self):
        return self.config.groups[self.group]

    @property
    def speed(self):
        return self.config.max_speed * self.config.speed_levels[self.level]

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
            self.group = (self.group + 1) % len(self.config.groups)
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
        self.enabled, self.note, self.warning = True, '', False

    def disable(self, measured, note, warning=False):
        self.enabled, self.note, self.warning = False, note, warning
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.targets.update({joint: measured[joint] for joint in self.limits if joint in measured})

    def move(self, axes, homing, dt):
        config = self.config
        desired = dict.fromkeys(self.limits, 0.0)
        if homing:
            for joint, (lower, upper) in self.limits.items():
                if joint != config.tool_joint:
                    error = clamp(0.0, lower, upper) - self.targets[joint]
                    desired[joint] = clamp(config.home_gain * error, -self.speed, self.speed)
        else:
            group = self.active_group
            for joint, axis, scale in zip(group.joints, group.axes, group.scales):
                desired[joint] += scale * self.stick(axes, axis) * self.speed
        desired[config.tool_joint] = config.tool_speed * (
            self.trigger(axes, config.tool_close_axis) - self.trigger(axes, config.tool_open_axis))
        step = config.max_accel * dt
        self.blocked = []
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
