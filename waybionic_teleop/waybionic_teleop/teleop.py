"""Turn Xbox controller input into arm joint targets (pure logic, no ROS)."""

from dataclasses import dataclass
import math

from waybionic_teleop.gamepad import AXIS, BUTTON

ACTIONS = ('enable', 'stop', 'group', 'home', 'faster', 'slower', 'lock', 'tilt_up', 'tilt_down')
CARTESIAN_AXES = ('x', 'y', 'z', 'roll')
INCISION_AXES = ('insert', 'pivot', 'roll')
MOVES = {'cartesian': CARTESIAN_AXES, 'incision': INCISION_AXES}
# How far the tool axis may pass from the incision point before the incision group stops.
INCISION_TOLERANCE = 0.002
# A tip past the incision point by more than solver rounding is inside the body.
INSERTED = 1e-9
INCISION_LOST = ('The tool is off the incision point; press Y, withdraw it in another group, '
                 'then choose the incision group again')
INCISION_HELD = 'The tool is inserted; withdraw it to the incision point before pressing Y'


def clamp(value, low, high):
    return min(max(value, low), high)


@dataclass
class Group:
    """Joints that share the sticks; each joint follows one axis with a signed gain."""

    name: str
    joints: list
    axes: list
    scales: list
    # 'cartesian' groups move the tool tip along x, y, z and roll instead of single joints;
    # 'incision' groups insert along the tool axis and tilt about the incision point.
    mode: str = 'joint'

    def describe(self):
        moves = MOVES.get(self.mode, self.joints)
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
    check_config(config)
    return config


def check_config(config):
    """Raise ValueError unless the controller mapping and motion limits are usable."""
    if not 0 < config.period < math.inf:
        raise ValueError('the control period (1 / rate_hz) must be positive and finite')
    if not config.groups:
        raise ValueError('at least one teleop group is required')
    speeds = (config.tool_speed, config.max_speed, config.max_accel, config.linear_speed,
              config.linear_accel, config.tilt_speed, config.home_gain)
    if any(not math.isfinite(value) or value <= 0 for value in speeds):
        raise ValueError('teleop speeds and accelerations must be positive and finite')
    if any(not math.isfinite(value) or not 0 < value <= 1 for value in config.speed_levels):
        raise ValueError('speed_levels must be in (0, 1]')
    for group in config.groups:
        # A scale above 1 would let a full stick pass the speed limits.
        if any(not -1.0 <= scale <= 1.0 for scale in group.scales):
            raise ValueError(f'group {group.name} scales must be between -1 and 1')
        if group.mode in MOVES:
            moves = ', '.join(MOVES[group.mode])
            if not len(group.axes) == len(group.scales) == len(MOVES[group.mode]):
                raise ValueError(f'group {group.name} needs {moves} axes and scales')
        elif group.mode != 'joint':
            raise ValueError(f'group {group.name} mode must be joint, cartesian or incision')
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


class ArmTeleop:
    """Hold joint targets and move them with the active group's sticks while enabled."""

    def __init__(self, config, limits, kinematics=None, collision=None):
        """Take the config, {joint: (lower, upper)} radians, ArmKinematics and ArmCollision."""
        check_config(config)
        self.config = config
        self.kinematics = kinematics
        self.collision = collision
        # Without kinematics for this arm, the Cartesian and incision groups are left out.
        self.groups = [group for group in config.groups
                       if group.mode not in MOVES or kinematics is not None]
        if not self.groups:
            raise ValueError('no joint-space group or compatible Cartesian kinematics')
        for group in self.groups:
            if group.mode in MOVES and (
                    tuple(group.joints) != kinematics.joints
                    or any(joint not in limits for joint in kinematics.joints)):
                raise ValueError(
                    f'{group.mode.capitalize()} group {group.name} does not match the URDF chain')
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
        self.insert = 0.0
        # Where the tool enters the body: the tip position when the incision group took over.
        self.incision = None
        # Whether the tool was out of the body when it last left the incision group.
        self.withdrawn = True
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
            self.change_group()
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

    def change_group(self):
        """Select the next group, unless the tool is inserted through the incision point."""
        if self.active_group.mode == 'incision' and self.incision is not None:
            miss, depth = self.incision_offset(self.targets)
            if miss <= INCISION_TOLERANCE and depth > INSERTED:
                # The other groups, and going home, could drag the tool sideways in the incision.
                self.note, self.warning = INCISION_HELD, True
                return
            self.withdrawn = depth <= INSERTED
        self.group = (self.group + 1) % len(self.groups)
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        self.wait_for_center = True
        if self.active_group.mode == 'incision' and self.incision is not None:
            withdrawn = self.withdrawn or self.incision_offset(self.targets)[1] <= INSERTED
            # Keep the incision point while the tool axis still passes it. Otherwise only a
            # withdrawn tool takes a new one, at its tip; one still inside stays stopped.
            if not self.align_incision() and withdrawn:
                self.incision = None

    def motion_input_held(self, axes):
        config = self.config
        sticks = {axis for group in config.groups for axis in group.axes}
        buttons = {config.buttons[action] for action in ('home', 'tilt_up', 'tilt_down')}
        return (any(self.stick(axes, axis) for axis in sticks)
                or any(self.trigger(axes, axis)
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
        if self.active_group.mode == 'incision' and self.incision is not None:
            self.align_incision()

    def disable(self, measured, note, warning=False):
        self.enabled, self.note, self.warning = False, note, warning
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.stop_cartesian()
        self.wait_for_center = False
        self.targets.update({joint: measured[joint] for joint in self.limits
                             if joint in measured and math.isfinite(measured[joint])})
        self.command_targets = dict(self.targets)

    def stop_cartesian(self):
        self.linear, self.tilt, self.roll, self.insert = (0.0, 0.0, 0.0), 0.0, 0.0, 0.0

    def move(self, axes, homing, dt):
        config = self.config
        self.command_targets = dict(self.targets)
        desired = dict.fromkeys(self.limits, 0.0)
        self.blocked = []
        before = dict(self.targets)
        # Going home would drag the tool sideways through the incision.
        incision = self.active_group.mode == 'incision'
        if homing and not incision:
            self.stop_cartesian()
            for joint, (lower, upper) in self.limits.items():
                if joint != config.tool_joint:
                    error = clamp(0.0, lower, upper) - self.targets[joint]
                    desired[joint] = clamp(config.home_gain * error, -self.speed, self.speed)
        elif self.active_group.mode == 'cartesian':
            self.jog(axes, dt)
            desired = {config.tool_joint: 0.0}
        elif incision:
            self.incise(axes, dt)
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
        self.avoid_collisions(before)

    def avoid_collisions(self, before):
        """Undo this step's arm motion if it would take a link into the table or the base."""
        if self.collision is None:
            return
        intrusions = self.collision.check(self.targets)
        if not intrusions:
            return
        # Backing out of a collision is allowed, but every contact has to be measured on its
        # own: a total would let one link press further in while another one pulls clear.
        # The relative slack, for depths and volumes alike, is rounding noise, not a margin.
        was = self.collision.check(before)
        worse = [hit for hit, amount in intrusions.items()
                 if amount > was.get(hit, 0.0) * (1.0 + 1e-9)]
        if not worse:
            return
        for joint, position in before.items():
            if joint != self.config.tool_joint:
                # Revert the published lookahead too, so no command continues into the contact.
                self.targets[joint] = self.command_targets[joint] = position
                self.velocities[joint] = 0.0
        self.stop_cartesian()
        self.blocked += worse

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

    def incise(self, axes, dt):
        """
        Slide the tool along its own axis, or tilt it about the incision point.

        The incision point stays on the tool axis. Five joints keep the axis in the arm's
        vertical plane, so the tool tilts only in that plane.
        """
        config, group, kinematics = self.config, self.active_group, self.kinematics
        insert, pivot, roll = [scale * self.stick(axes, axis)
                               for axis, scale in zip(group.axes, group.scales)]
        linear_step, step = config.linear_accel * dt, config.max_accel * dt
        self.insert += clamp(self.linear_speed * insert - self.insert, -linear_step, linear_step)
        # Pushing up tilts the tool axis towards straight up, like the tilt-up button.
        tilt = -config.tilt_speed * config.speed_levels[self.level] * pivot
        self.tilt += clamp(tilt - self.tilt, -step, step)
        self.roll += clamp(roll * self.speed - self.roll, -step, step)
        before = {joint: self.targets[joint] for joint in kinematics.joints}
        if self.incision is None:
            # Selecting the group again is the documented way out of INCISION_LOST, and it
            # takes the tool's own tip as the new incision point. Clear that warning, and
            # only that one, so a later warning is never hidden by this.
            self.incision = kinematics.forward(before)[0]
            if self.note == INCISION_LOST:
                self.note, self.warning = '', False
        miss, depth = self.incision_offset(before)
        if miss > INCISION_TOLERANCE:
            # Re-enabled or reselected away from the incision point: never pull it back there.
            self.insert = self.tilt = self.roll = 0.0
            self.blocked = ['incision']
            self.note, self.warning = INCISION_LOST, True
            return
        if self.note == INCISION_HELD and depth <= INSERTED:
            self.note, self.warning = '', False
        if not (self.insert or self.tilt or self.roll):
            # Hold the pose exactly: solving for it again would only add rounding noise.
            self.velocities.update(dict.fromkeys(kinematics.joints, 0.0))
            return
        after, fraction, self.blocked = self.incision_step(before, dt)
        self.insert, self.tilt, self.roll = (
            self.insert * fraction, self.tilt * fraction, self.roll * fraction)
        for joint in kinematics.joints:
            self.velocities[joint] = (after[joint] - before[joint]) / dt
            self.targets[joint] = after[joint]
        if self.insert or self.tilt or self.roll:
            # The next period's pose on the same path, so the axis still passes the incision.
            future, _, _ = self.incision_step(after, config.period)
        else:
            future = after
        self.command_targets.update(future)

    def incision_offset(self, joints):
        """Return how far the tool axis misses the incision point and the tip's depth past it."""
        tip, _ = self.kinematics.forward(joints)
        offset = [a - b for a, b in zip(tip, self.incision)]
        depth = sum(a * b for a, b in zip(offset, self.kinematics.axis(joints)))
        return math.sqrt(max(sum(a * a for a in offset) - depth * depth, 0.0)), depth

    def align_incision(self):
        """Move the incision point, never the arm, onto the tool axis; False if it is too far."""
        miss, depth = self.incision_offset(self.targets)
        if miss > INCISION_TOLERANCE:
            return False
        tip, _ = self.kinematics.forward(self.targets)
        self.incision = tuple(point - depth * direction for point, direction
                              in zip(tip, self.kinematics.axis(self.targets)))
        return True

    def incision_step(self, joints, dt):
        """Step dt along the insert and tilt path through the incision, as kinematics.jog does."""
        kinematics = self.kinematics
        tip, pitch = kinematics.forward(joints)
        _, depth = self.incision_offset(joints)
        # Aim for the point on the new axis through the incision, at the new depth, so the
        # incision never drifts off the axis.
        target = [point + (depth + self.insert * dt) * direction for point, direction
                  in zip(self.incision, kinematics.axis(joints, pitch + self.tilt * dt))]
        velocity = [(goal - now) / dt for goal, now in zip(target, tip)]
        return kinematics.jog(joints, velocity, self.tilt, self.roll, dt, self.limits,
                              self.config.max_speed)

    def stick(self, axes, name):
        value = self.axis(axes, name)
        magnitude = abs(value) - self.config.deadzone
        if magnitude <= 0:
            return 0.0
        return math.copysign(min(magnitude / (1.0 - self.config.deadzone), 1.0), value)

    def trigger(self, axes, name):
        # game_controller_node triggers rest at 0 and reach -1 when fully pressed.
        magnitude = clamp(-self.axis(axes, name), 0.0, 1.0) - self.config.deadzone
        if magnitude <= 0:
            return 0.0
        return min(magnitude / (1.0 - self.config.deadzone), 1.0)

    @staticmethod
    def axis(axes, name):
        index = AXIS[name]
        value = float(axes[index]) if index < len(axes) else 0.0
        return value if math.isfinite(value) else 0.0
