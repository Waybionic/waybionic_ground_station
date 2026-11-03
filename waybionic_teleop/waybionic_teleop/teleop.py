"""Turn Xbox controller input into arm joint targets (pure logic, no ROS)."""

from dataclasses import dataclass
import itertools
import math

from waybionic_teleop.gamepad import AXIS, BUTTON
from waybionic_teleop.wander import RAMP_PEAK, Wander

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
# Chicken head mode. The tip is held at a spot while the sticks tilt and roll the tool.
HEAD_ARRIVED = 1e-6
# The share of a drift of the held tip that each step takes back, so rounding never builds up.
HEAD_PULL = 0.5
# An approach that has made no progress for this long is reported as blocked.
HEAD_BLOCKED_S = 1.5
# A tilt or roll that is asked for, but gets less than this share of its step, is stopped by a
# limit, though the kinematics does not always say which.
HEAD_STALLED = 1e-3
HEAD_NEAR_LIMIT = math.radians(2.0)
HEAD_STATES = ('off', 'approach', 'hold', 'blocked', 'unavailable')
HEAD_UNAVAILABLE = ('Chicken head needs the arm model and a chicken_head section in '
                    'xbox_teleop.yaml')
# Demo mode. The arm swings slowly about the pose it started from, and comes back to it.
DEMO_STATES = ('off', 'running', 'stopping', 'unavailable')
DEMO_UNAVAILABLE = 'Demo mode needs a demo section in xbox_teleop.yaml that fits this arm'
# The shares of its swing that demo mode tries, biggest first, until the extremes of the swing
# are clear of the table and the base.
DEMO_SCALES = (1.0, 0.6, 0.35)
WAIT_NOTE = 'Center motion controls before moving in the new group'
HAND_BACK_NOTE = 'Center motion controls to take over from the mode that just ended'
INCISION_JOINT_OFF = ('A joint is switched off with the tool through the incision point; switch '
                      'it back on in Joint control to withdraw the tool')


def clamp(value, low, high):
    return min(max(value, low), high)


def axis_words(axis):
    """Return what to call a controller axis, for example 'left stick' for 'left_y'."""
    side, _, kind = axis.rpartition('_')
    return f'{side} stick' if kind in ('x', 'y') else axis.replace('_', ' ')


@dataclass
class HeadMode:
    """Chicken head mode while it is on: approaching its goal joints, then holding the tip."""

    goal: dict
    # The tip position being held, once the tool has arrived there.
    anchor: tuple = None
    # How fast the joint furthest from its goal is going (rad/s), and how long it has been stuck.
    rate: float = 0.0
    stuck: float = 0.0


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
class HeadConfig:
    """Where chicken head mode holds the tool tip, and the sticks that tilt and roll the tool."""

    # The tip in the base frame (m), and the tool axis angle from straight up (rad).
    position: tuple
    pitch: float
    tilt_axis: str
    roll_axis: str
    tilt_scale: float
    roll_scale: float


@dataclass
class DemoConfig:
    """How demo mode swings the arm's joints about the pose it starts from."""

    joints: list
    # Per joint: how far it swings each way (rad), whole swings per loop, and where in its swing
    # it starts (rad).
    amplitudes: list
    cycles: list
    phases: list
    # Seconds of demo time for one loop, and for the swing to grow or shrink.
    loop: float
    ramp: float
    # How far a joint stays from its limits, and the least swing worth making (rad).
    margin: float
    min_swing: float
    # The fastest any joint may go (rad/s).
    max_speed: float


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
    head: HeadConfig = None
    demo: DemoConfig = None


def head_from_parameters(params):
    """Build the HeadConfig from the chicken_head parameters, or None if there are none."""
    if 'chicken_head.position' not in params:
        return None
    axes, scales = params['chicken_head.axes'], params['chicken_head.scales']
    if len(axes) != 2 or len(scales) != 2:
        raise ValueError('chicken_head axes and scales need a tilt and a roll entry')
    return HeadConfig(
        position=tuple(float(value) for value in params['chicken_head.position']),
        pitch=math.radians(float(params['chicken_head.pitch_deg'])),
        tilt_axis=axes[0], roll_axis=axes[1],
        tilt_scale=float(scales[0]), roll_scale=float(scales[1]))


def demo_from_parameters(params):
    """Build the DemoConfig from the demo parameters, or None if there are none."""
    if 'demo.joints' not in params:
        return None
    return DemoConfig(
        joints=list(params['demo.joints']),
        amplitudes=[math.radians(float(value)) for value in params['demo.amplitudes_deg']],
        # A whole number that arrived as a float is still a whole number.
        cycles=[int(value) if isinstance(value, float) and value.is_integer() else value
                for value in params['demo.cycles']],
        phases=[math.radians(float(value)) for value in params['demo.phases_deg']],
        loop=float(params['demo.loop_s']), ramp=float(params['demo.ramp_s']),
        margin=math.radians(float(params['demo.margin_deg'])),
        min_swing=math.radians(float(params['demo.min_swing_deg'])),
        max_speed=math.radians(float(params['demo.max_speed_deg_s'])))


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
            home_gain=float(params['home_gain']),
            head=head_from_parameters(params),
            demo=demo_from_parameters(params))
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
    head = config.head
    if head is not None:
        if len(head.position) != 3 or not all(math.isfinite(value) for value in head.position):
            raise ValueError('chicken_head position must be three finite numbers (x, y, z in m)')
        if not math.isfinite(head.pitch):
            raise ValueError('chicken_head pitch_deg must be finite')
        if any(not -1.0 <= scale <= 1.0 for scale in (head.tilt_scale, head.roll_scale)):
            raise ValueError('chicken_head scales must be between -1 and 1')
    if config.demo is not None:
        check_demo(config, config.demo)
    unknown = [name for name in [axis for group in config.groups for axis in group.axes]
               + [config.tool_close_axis, config.tool_open_axis]
               + ([head.tilt_axis, head.roll_axis] if head is not None else [])
               if name not in AXIS]
    unknown += [name for name in config.buttons.values() if name not in BUTTON]
    if unknown:
        raise ValueError('unknown controller inputs: ' + ', '.join(map(str, unknown)))
    if not 0 <= config.speed_level < len(config.speed_levels) or not 0 <= config.deadzone < 1:
        raise ValueError('initial_speed_level or deadzone out of range')
    if len(config.tool_limits) != 2 or not config.tool_limits[0] < config.tool_limits[1]:
        raise ValueError('tool_limits must be [open, closed] with open < closed')


def check_demo(config, demo):
    """Raise ValueError unless demo mode's swings are usable and slow enough."""
    if not demo.joints or len({len(demo.joints), len(demo.amplitudes), len(demo.cycles),
                               len(demo.phases)}) != 1:
        raise ValueError('demo needs a joint, an amplitude, a cycle count and a phase for '
                         'each swing')
    if len(set(demo.joints)) != len(demo.joints) or config.tool_joint in demo.joints:
        raise ValueError('demo joints must be different arm joints, not the tool')
    numbers = [*demo.amplitudes, *demo.phases, demo.loop, demo.ramp, demo.margin, demo.min_swing,
               demo.max_speed]
    if any(not math.isfinite(number) for number in numbers):
        raise ValueError('demo settings must be finite numbers')
    if any(amplitude < 0 for amplitude in demo.amplitudes) or demo.margin < 0:
        raise ValueError('demo amplitudes_deg and margin_deg must not be negative')
    if any(value <= 0 for value in (demo.loop, demo.ramp, demo.min_swing, demo.max_speed)):
        raise ValueError('demo loop_s, ramp_s, min_swing_deg and max_speed_deg_s must be positive')
    if any(isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1
           for cycles in demo.cycles):
        raise ValueError('demo cycles must be whole numbers of at least 1')
    if demo.max_speed > config.max_speed:
        raise ValueError('demo max_speed_deg_s must not be above max_speed_deg_s')
    for joint, amplitude, cycles in zip(demo.joints, demo.amplitudes, demo.cycles):
        # The swing's own speed, plus the swing growing or shrinking over the ramp.
        fastest = amplitude * (2.0 * math.pi * cycles / demo.loop + RAMP_PEAK / demo.ramp)
        if fastest > demo.max_speed:
            raise ValueError(
                f'demo {joint} could go at {math.degrees(fastest):.1f} deg/s, above '
                f'max_speed_deg_s {math.degrees(demo.max_speed):.1f}')


class ArmTeleop:
    """Hold joint targets and move them with the active group's sticks while enabled."""

    def __init__(self, config, limits, kinematics=None, collision=None, locked=None):
        """
        Take the config, {joint: (lower, upper)} radians, ArmKinematics and ArmCollision.

        locked gives {joint: angle} for joints that cannot move on this arm, which then stay
        at that angle as if it were both of their limits.
        """
        check_config(config)
        self.config = config
        self.kinematics = kinematics
        self.collision = collision
        # Without kinematics for this arm, the Cartesian and incision groups are left out.
        self.configured_groups = [group for group in config.groups
                                  if group.mode not in MOVES or kinematics is not None]
        if not self.configured_groups:
            raise ValueError('no joint-space group or compatible Cartesian kinematics')
        for group in self.configured_groups:
            if group.mode in MOVES and (
                    tuple(group.joints) != kinematics.joints
                    or any(joint not in limits for joint in kinematics.joints)):
                raise ValueError(
                    f'{group.mode.capitalize()} group {group.name} does not match the URDF chain')
        self.limits = dict(limits)
        self.limits[config.tool_joint] = config.tool_limits
        self.locked = {joint: angle for joint, angle in (locked or {}).items()
                       if joint in self.limits}
        for joint, angle in self.locked.items():
            self.limits[joint] = (angle, angle)
        # Joints the operator switched off: each holds the angle it had when teleop was enabled.
        self.disabled = set()
        self.disabled_angles = {}
        self.groups = self.usable_groups()
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
        # The sticks and triggers in the last controller sample, for a mode that ends between
        # samples.
        self.last_axes = ()
        self.wait_for_center = False
        self.wait_note = WAIT_NOTE
        # Chicken head mode, while it is on: see set_head_mode.
        self.head = None
        # Demo mode, while it is on: a Wander. See set_demo_mode.
        self.demo = None
        # Why demo mode last stopped by itself, until it is switched on again.
        self.demo_ended = ''
        self.demo_misfit = config.demo is None or any(
            joint not in self.limits for joint in config.demo.joints)
        self.blocked = []
        self.note = 'Press Start (Xbox Menu button) to enable'
        self.warning = False

    @property
    def active_group(self):
        return self.groups[self.group]

    def movable(self, joint):
        """Return whether the sticks can move this joint: it is neither locked nor disabled."""
        return joint not in self.locked and joint not in self.disabled

    def group_usable(self, group):
        """Return whether a group has something to move: a Cartesian solve needs every joint."""
        if group.mode in MOVES:
            return all(self.movable(joint) for joint in group.joints)
        return any(self.movable(joint) for joint in group.joints)

    def usable_groups(self, keep=None):
        """Return the groups with something to move, and keep, which is not left for one."""
        return [group for group in self.configured_groups
                if group is keep or self.group_usable(group)] or [self.configured_groups[0]]

    def inserted(self):
        """Return whether the tool is through the incision point, which groups may not leave."""
        if self.active_group.mode != 'incision' or self.incision is None or not self.targets:
            return False
        miss, depth = self.incision_offset(self.targets)
        return miss <= INCISION_TOLERANCE and depth > INSERTED

    def set_disabled(self, joints, measured):
        """
        Switch off exactly these joints; return True if that changed anything.

        A disabled joint never moves, so the arm can run without a joint whose drive is faulty
        or missing. While enabled, teleop stops where the arm is: the next Start enables the
        remaining joints from the measured pose.
        """
        joints = {joint for joint in joints if joint in self.limits and joint not in self.locked}
        if joints == self.disabled:
            return False
        if self.enabled:
            self.disable(measured, 'Joint enable changed; press Start (Xbox Menu button) to '
                         'enable')
        previous = self.active_group
        # The other groups, and going home, could drag the tool sideways in the incision, which
        # Y and A are refused for, so a tool that is through it stays in its group, unable to move
        # until the joint is switched back on.
        keep = previous if self.inserted() else None
        self.disabled = joints
        self.disabled_angles = {joint: angle for joint, angle in self.disabled_angles.items()
                                if joint in joints}
        self.groups = self.usable_groups(keep)
        self.group = next((index for index, group in enumerate(self.groups)
                           if group.name == previous.name), 0)
        if self.active_group is not previous and previous.mode == 'incision' and (
                self.incision is not None):
            self.withdrawn = self.incision_offset(self.targets)[1] <= INSERTED
        if keep is not None and not self.group_usable(keep):
            self.note, self.warning = INCISION_JOINT_OFF, True
        return True

    @property
    def speed(self):
        return self.config.max_speed * self.config.speed_levels[self.level]

    @property
    def linear_speed(self):
        return self.config.linear_speed * self.config.speed_levels[self.level]

    def update(self, axes, buttons, measured, dt):
        """Apply one controller sample; return True when the targets should be sent."""
        self.last_axes = axes
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
                self.note = self.wait_note
                self.warning = True
                return True
            self.wait_for_center = False
            self.note, self.warning = '', False
        self.move(axes, self.config.buttons['home'] in held, dt)
        return True

    def change_group(self):
        """Select the next group, unless the tool is inserted through the incision point."""
        if self.active_group.mode == 'incision' and self.incision is not None:
            if self.inserted():
                # The other groups, and going home, could drag the tool sideways in the incision.
                self.note, self.warning = INCISION_HELD, True
                return
            self.withdrawn = self.incision_offset(self.targets)[1] <= INSERTED
        self.end_head()
        self.end_demo()
        self.group = (self.group + 1) % len(self.groups)
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        self.wait_for_center = True
        self.wait_note = WAIT_NOTE
        if self.active_group.mode == 'incision' and self.incision is not None:
            withdrawn = self.withdrawn or self.incision_offset(self.targets)[1] <= INSERTED
            # Keep the incision point while the tool axis still passes it. Otherwise only a
            # withdrawn tool takes a new one, at its tip; one still inside stays stopped.
            if not self.align_incision() and withdrawn:
                self.incision = None

    def hand_back(self):
        """After a mode ends, hold the arm until the controls are centred, if they are held."""
        if self.motion_input_held(self.last_axes):
            self.wait_for_center = True
            self.wait_note = HAND_BACK_NOTE

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
        if not any(self.movable(joint) for joint in self.limits):
            self.note = 'No joint is enabled; switch one on in the Joint control panel'
            self.warning = True
            return
        if not self.group_usable(self.active_group):
            self.note, self.warning = INCISION_JOINT_OFF, True
            return
        if self.motion_input_held(axes):
            # A stuck or held input must never start moving the arm the moment it is enabled.
            self.note = 'Center the sticks and release the triggers and buttons, then press Start'
            self.warning = True
            return
        # Start from the measured pose so enabling never makes the arm jump.
        self.targets = {joint: measured[joint] for joint in self.limits}
        self.disabled_angles = {joint: measured[joint] for joint in self.disabled}
        self.command_targets = dict(self.targets)
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.pin_locked()
        self.stop_cartesian()
        self.wait_for_center = False
        self.head = None
        self.demo, self.demo_ended = None, ''
        self.enabled, self.note, self.warning = True, '', False
        if self.active_group.mode == 'incision' and self.incision is not None:
            self.align_incision()

    def disable(self, measured, note, warning=False):
        self.enabled, self.note, self.warning = False, note, warning
        self.head = None
        self.demo, self.demo_ended = None, ''
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.stop_cartesian()
        self.wait_for_center = False
        self.targets.update({joint: measured[joint] for joint in self.limits
                             if joint in measured and math.isfinite(measured[joint])})
        self.command_targets = dict(self.targets)
        self.pin_locked()

    def pin_locked(self):
        # A solve may leave a locked joint a rounding error off its angle; send it unchanged.
        for joint, angle in {**self.disabled_angles, **self.locked}.items():
            self.targets[joint] = self.command_targets[joint] = angle
            self.velocities[joint] = 0.0

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
            self.end_head()
            self.end_demo()
            self.stop_cartesian()
            for joint, (lower, upper) in self.limits.items():
                if joint != config.tool_joint and self.movable(joint):
                    error = clamp(0.0, lower, upper) - self.targets[joint]
                    desired[joint] = clamp(config.home_gain * error, -self.speed, self.speed)
        elif self.head is not None:
            self.head_step(axes, dt)
            desired = {config.tool_joint: 0.0}
        elif self.demo is not None:
            desired.update(self.demo_step(dt))
        elif self.active_group.mode == 'cartesian':
            self.jog(axes, dt)
            desired = {config.tool_joint: 0.0}
        elif incision:
            self.incise(axes, dt)
            desired = {config.tool_joint: 0.0}
        else:
            group = self.active_group
            for joint, axis, scale in zip(group.joints, group.axes, group.scales):
                # A joint that cannot move is not a limit to hit, so its stick does nothing.
                if self.movable(joint):
                    desired[joint] += scale * self.stick(axes, axis) * self.speed
        desired[config.tool_joint] = 0.0
        if self.movable(config.tool_joint):
            desired[config.tool_joint] = config.tool_speed * (
                self.trigger(axes, config.tool_close_axis)
                - self.trigger(axes, config.tool_open_axis))
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
        if self.head is not None and self.head.anchor is None:
            # An approach that a contact keeps stopping starts again slowly, and is reported.
            self.head.stuck = self.head.stuck + dt if self.blocked else 0.0
            if self.blocked:
                self.head.rate = 0.0
        if self.demo is not None:
            self.demo_settle()
        self.pin_locked()

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

    def head_status(self):
        """Return chicken head mode's state, one of HEAD_STATES, and a line saying why."""
        if self.kinematics is None or self.config.head is None:
            return 'unavailable', HEAD_UNAVAILABLE
        head = self.head
        if head is None:
            return 'off', ''
        if head.anchor is not None:
            settings = self.config.head
            return 'hold', (f'Holding the tip in place: the {axis_words(settings.tilt_axis)} '
                            f'tilts the tool, the {axis_words(settings.roll_axis)} rolls it')
        if head.stuck >= HEAD_BLOCKED_S:
            return 'blocked', ('The way to the head position is blocked by '
                               + ', '.join(self.blocked or ['something']))
        return 'approach', 'Moving the tip to the head position'

    def head_problem(self):
        """Return why chicken head mode cannot start now, or '' if it can; set head_goal."""
        if self.kinematics is None or self.config.head is None:
            return HEAD_UNAVAILABLE
        if not self.enabled:
            return 'Press Start (Xbox Menu button) first, then switch Chicken head on'
        if self.demo is not None:
            return 'Switch Demo mode off first'
        if self.inserted():
            return INCISION_HELD
        stuck = [joint for joint in self.kinematics.joints if not self.movable(joint)]
        if stuck:
            return (f'{", ".join(stuck)} cannot move (locked or switched off); chicken head '
                    'needs every arm joint')
        kinematics, head = self.kinematics, self.config.head
        roll = kinematics.joints[4]
        arm = {name: bounds for name, bounds in self.limits.items()
               if name in kinematics.joints and name != roll}
        goal = kinematics.inverse(head.position, head.pitch, self.targets[roll], self.targets, arm)
        if goal is None:
            return 'The head position is out of reach for the arm and its joint limits'
        self.head_goal = goal
        return ''

    def set_head_mode(self, enabled):
        """Switch chicken head mode on or off; return (whether it is as asked, what to tell)."""
        if not enabled:
            if self.head is not None:
                self.end_head('Chicken head off: the arm holds where it is', hand_back=True)
            return True, 'Chicken head is off'
        if self.head is not None:
            return True, 'Chicken head is already on'
        problem = self.head_problem()
        if problem:
            return False, problem
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        self.head = HeadMode(self.head_goal)
        self.note, self.warning = 'Chicken head: moving the tip to the head position', False
        return True, 'Chicken head is on: moving the tip to the head position'

    def end_head(self, note='', hand_back=False):
        """
        Leave chicken head mode, if it is on, holding the arm where it is.

        With ``hand_back``, the sticks that tilt and roll the tool must be centred before the
        active group's sticks move the arm.
        """
        if self.head is None:
            return
        self.head = None
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        if note:
            self.note, self.warning = note, False
        if hand_back:
            self.hand_back()

    def head_step(self, axes, dt):
        """Take one step of chicken head mode: toward the head spot, or holding it."""
        if self.head.anchor is None:
            self.head_approach(dt)
        else:
            self.head_hold(axes, dt)

    def head_approach(self, dt):
        """Move every arm joint in a straight line, in joint space, to the goal pose."""
        head, config = self.head, self.config
        joints = self.kinematics.joints
        error = {joint: head.goal[joint] - self.targets[joint] for joint in joints}
        distance = max(abs(value) for value in error.values())
        if distance <= HEAD_ARRIVED:
            self.targets.update(head.goal)
            self.command_targets.update(head.goal)
            for joint in joints:
                self.velocities[joint] = 0.0
            head.anchor = self.kinematics.forward(head.goal)[0]
            self.wait_for_center = True
            self.wait_note = 'Center motion controls before tilting or rolling the head'
            return
        # Speed up at the acceleration limit and slow down so that every joint stops together.
        head.rate = min(head.rate + config.max_accel * dt, self.speed,
                        math.sqrt(2.0 * config.max_accel * distance))
        step = min(head.rate * dt, distance)
        remaining = distance - step
        ahead = min(head.rate * config.period, remaining)
        for joint in joints:
            self.targets[joint] += error[joint] * step / distance
            self.velocities[joint] = error[joint] * step / distance / dt
            self.command_targets[joint] = self.targets[joint] + error[joint] * ahead / distance

    def nearest_limit(self):
        """Name the arm joint closest to one of its limits, or 'reach' if none is close."""
        margins = {joint: min(self.targets[joint] - self.limits[joint][0],
                              self.limits[joint][1] - self.targets[joint])
                   for joint in self.kinematics.joints}
        joint = min(margins, key=margins.get)
        return [joint] if margins[joint] < HEAD_NEAR_LIMIT else ['reach']

    def head_hold(self, axes, dt):
        """Tilt and roll the tool about the held tip, with the tip pulled back to the spot."""
        config, kinematics, head = self.config, self.kinematics, self.head
        settings = config.head
        if dt <= 0:
            return
        top = config.tilt_speed * config.speed_levels[self.level]
        # Stick up tilts the tool up, which lowers the pitch angle, as the tilt buttons do.
        tilt = clamp(
            -settings.tilt_scale * self.stick(axes, settings.tilt_axis) * top
            + top * ((config.buttons['tilt_down'] in self.held)
                     - (config.buttons['tilt_up'] in self.held)), -top, top)
        roll = settings.roll_scale * self.stick(axes, settings.roll_axis) * self.speed
        step = config.max_accel * dt
        tilt = self.tilt + clamp(tilt - self.tilt, -step, step)
        roll = self.roll + clamp(roll - self.roll, -step, step)
        before = {joint: self.targets[joint] for joint in kinematics.joints}
        tip, _ = kinematics.forward(before)
        pull = [HEAD_PULL * (goal - now) / dt for goal, now in zip(head.anchor, tip)]
        after, fraction, self.blocked = kinematics.jog(
            before, pull, tilt, roll, dt, self.limits, config.max_speed)
        if not self.blocked and fraction < HEAD_STALLED and (abs(tilt) > 1e-6 or abs(roll) > 1e-6):
            self.blocked = self.nearest_limit()
        self.tilt, self.roll = tilt * fraction, roll * fraction
        for joint in kinematics.joints:
            self.velocities[joint] = (after[joint] - before[joint]) / dt
            self.targets[joint] = after[joint]
        if self.tilt or self.roll:
            future, _, _ = kinematics.jog(
                after, (0.0, 0.0, 0.0), self.tilt, self.roll, config.period, self.limits,
                config.max_speed)
        else:
            future = after
        self.command_targets.update(future)

    def demo_status(self):
        """Return demo mode's state, one of DEMO_STATES, and a line saying what is going on."""
        if self.demo_misfit:
            return 'unavailable', DEMO_UNAVAILABLE
        if self.demo is None:
            return 'off', self.demo_ended
        if self.demo.ramp.goal == 0.0:
            return 'stopping', 'Returning to the pose it started from'
        return 'running', 'Moving slowly around its start pose; B stops it at once'

    def demo_plan(self):
        """Return ({joint: (amplitude, cycles, phase)}, '') for a swing that fits, or why not."""
        if self.demo_misfit:
            return {}, DEMO_UNAVAILABLE
        if not self.enabled:
            return {}, 'Press Start (Xbox Menu button) first, then switch Demo mode on'
        if self.head is not None:
            return {}, 'Switch Chicken head off first'
        if self.inserted():
            return {}, 'The tool is inserted; withdraw it to the incision point first'
        demo, swings = self.config.demo, {}
        for joint, amplitude, cycles, phase in zip(
                demo.joints, demo.amplitudes, demo.cycles, demo.phases):
            lower, upper = self.limits[joint]
            room = min(self.targets[joint] - lower, upper - self.targets[joint]) - demo.margin
            if self.movable(joint) and min(amplitude, room) >= demo.min_swing:
                swings[joint] = (min(amplitude, room), cycles, phase)
        if not swings:
            if any(self.movable(joint) for joint in demo.joints):
                return {}, ('No joint has room to swing here; move the arm away from its joint '
                            'limits first')
            return {}, 'No demo joint can move: they are locked or switched off'
        for scale in DEMO_SCALES:
            scaled = {joint: (amplitude * scale, cycles, phase)
                      for joint, (amplitude, cycles, phase) in swings.items()}
            hit = self.swing_hit(scaled)
            if not hit:
                return scaled, ''
        return {}, (f'Swinging here would take the arm into {hit}; move it away from the table '
                    'and the base first')

    def swing_hit(self, swings):
        """Name what the arm would touch at the extremes of the swings, or '' if nothing."""
        if self.collision is None:
            return ''
        joints = list(swings)
        reach = {joint: swings[joint][0] for joint in joints}
        # The start pose, each joint at either end of its swing, and every mix of the ends.
        offsets = [{}]
        offsets += [{joint: sign * reach[joint]} for joint in joints for sign in (-1.0, 1.0)]
        offsets += [{joint: sign * reach[joint] for joint, sign in zip(joints, signs)}
                    for signs in itertools.product((-1.0, 1.0), repeat=len(joints))]
        for offset in offsets:
            pose = {joint: angle + offset.get(joint, 0.0) for joint, angle in self.targets.items()}
            found = self.collision.check(pose)
            if found:
                return ', '.join(found)
        return ''

    def demo_tempo(self):
        """Return how fast the demo clock runs: 1 at the starting speed level, less below it."""
        levels = self.config.speed_levels
        return min(1.0, levels[self.level] / levels[self.config.speed_level])

    def set_demo_mode(self, enabled):
        """Switch demo mode on or off; return (whether it is as asked, what to tell)."""
        demo = self.demo
        if not enabled:
            if demo is None:
                return True, 'Demo mode is off'
            demo.ramp.aim(0.0)
            self.note, self.warning = 'Demo mode off: returning to the pose it started from', False
            return True, 'Demo mode is stopping: the arm is returning to the pose it started from'
        if demo is not None:
            demo.ramp.aim(1.0)
            self.note, self.warning = 'Demo mode: moving slowly around its start pose', False
            return True, 'Demo mode is on'
        swings, problem = self.demo_plan()
        if problem:
            return False, problem
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        settings = self.config.demo
        self.demo = Wander({joint: self.targets[joint] for joint in swings}, swings,
                           settings.loop, settings.ramp, self.demo_tempo())
        self.demo_ended = ''
        self.note, self.warning = 'Demo mode: moving slowly around its start pose', False
        return True, 'Demo mode is on: the arm is starting to move slowly'

    def end_demo(self, note='', warning=False, hand_back=False):
        """Leave demo mode, if it is on, holding the arm where it is."""
        if self.demo is None:
            return
        self.demo = None
        self.demo_ended = note if warning else ''
        self.stop_cartesian()
        self.velocities = dict.fromkeys(self.limits, 0.0)
        self.command_targets = dict(self.targets)
        if note:
            self.note, self.warning = note, warning
        if hand_back:
            self.hand_back()

    def demo_step(self, dt):
        """Advance the demo and return the joint velocities that follow its swing."""
        if dt <= 0:
            return {}
        self.demo.advance(dt, self.demo_tempo())
        cap = self.config.demo.max_speed
        return {joint: clamp((goal - self.targets[joint]) / dt, -cap, cap)
                for joint, goal in self.demo.positions().items()}

    def demo_settle(self):
        """End the demo if a step ran into something, or once its swing has shrunk to nothing."""
        stopped = [name for name in self.blocked if name != self.config.tool_joint]
        if stopped:
            self.end_demo('Demo stopped: the arm would hit ' + ', '.join(stopped),
                          warning=True, hand_back=True)
        elif self.demo.finished:
            # Back at the start pose, less the rounding of every step it took.
            self.targets.update(self.demo.centre)
            self.end_demo('Demo mode finished: the arm is back where it started',
                          hand_back=True)

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
