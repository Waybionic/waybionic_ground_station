"""Start-up, dropout and set zero of one drive, run against a simulated MKS drive."""

from waybionic_teleop import mks_can
from waybionic_teleop.drive_link import DriveLink, RETRY_S, SETUP
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

TICK = 0.01
TIMEOUT = 0.5


class Rig:
    """One drive link talking to one simulated drive, ticked like the node does."""

    def __init__(self):
        self.servo = SimulatedServo(1)
        self.bus = SimulatedBus([self.servo], 1000000)
        self.link = DriveLink(1, 500, TIMEOUT)
        self.now = 0.0
        self.sent = []

    def tick(self, count=1):
        for _ in range(count):
            self.now += TICK
            self.bus.step(TICK)
            if (data := self.link.poll(self.now)) is not None:
                self.sent.append(data[0])
                self.bus.send(1, data)
            while (reply := self.bus.receive()) is not None:
                self.link.on_reply(*mks_can.parse(*reply), self.now)


def test_setup_frames_go_out_in_order_each_after_its_confirmation():
    rig = Rig()
    rig.tick(6)
    assert rig.sent[:5] == [*SETUP, mks_can.READ_ENCODER]
    assert rig.link.ready
    assert (rig.servo.mode, rig.servo.enabled, rig.servo.heartbeat_ms) == (
        mks_can.MODE_SR_VFOC, True, 500)


def test_ready_only_after_the_encoder_read_answers():
    rig = Rig()
    rig.servo.axis = 1234.0
    rig.tick(4)
    assert (rig.link.phase, rig.link.count) == ('encoder', None)
    rig.tick()
    assert (rig.link.ready, rig.link.count) == (True, 1234)


def test_an_unanswered_setup_frame_is_resent_after_the_timeout():
    rig = Rig()
    rig.servo.unplugged = True
    rig.tick(round(TIMEOUT / TICK) + 2)
    assert rig.sent == [mks_can.SET_MODE, mks_can.SET_MODE]
    assert rig.link.describe() == 'no reply to 82h; setup 82h'


def test_a_refused_frame_is_retried_from_the_start_after_a_pause():
    rig = Rig()
    rig.tick(2)
    # The simulated drive never refuses, so the F3h refusal is handed to the link directly.
    assert rig.link.poll(rig.now) == mks_can.enable(1)
    rig.link.on_reply(mks_can.ENABLE, b'\x00', rig.now)
    assert rig.link.describe() == 'F3h refused, retrying'
    rig.sent.clear()
    rig.tick(round(RETRY_S / TICK) - 5)
    assert rig.sent == []
    rig.tick(15)
    assert rig.sent[:4] == list(SETUP) and rig.link.ready


def test_a_drive_that_stops_answering_starts_again_from_setup():
    rig = Rig()
    rig.tick(6)
    rig.servo.unplugged = True
    rig.tick(round(TIMEOUT / TICK) + 2)
    assert (rig.link.phase, rig.link.count, rig.link.dropouts) == ('setup', None, 1)
    assert rig.link.problem == 'no reply to 31h'
    rig.servo.unplugged = False
    rig.tick(round(TIMEOUT / TICK) + 6)
    assert rig.link.ready and rig.link.problem == ''


def test_set_zero_is_confirmed_then_the_encoder_is_read_again():
    rig = Rig()
    rig.servo.axis = 5000.0
    rig.tick(6)
    rig.link.zero()
    rig.sent.clear()
    rig.tick(3)
    assert rig.sent[:2] == [mks_can.SET_ZERO, mks_can.READ_ENCODER]
    assert (rig.link.ready, rig.link.zeroed, rig.link.count) == (True, True, 0)


def test_a_drive_lost_during_set_zero_is_reported_not_zeroed():
    rig = Rig()
    rig.tick(6)
    rig.servo.unplugged = True
    rig.link.zero()
    rig.tick(round(TIMEOUT / TICK) + 2)
    assert (rig.link.zeroed, rig.link.phase) == (False, 'setup')
