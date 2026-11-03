"""The demo's slow wandering: the ramp that starts and stops the swing, and the swing itself."""

import math

import pytest

from waybionic_teleop.wander import Ramp, TEMPO_RATE, Wander

DT = 1.0 / 120.0
JOINTS = ('joint_1', 'joint_2', 'joint_3')
CENTRE = {'joint_1': 0.1, 'joint_2': -0.2, 'joint_3': 0.3}
SWINGS = {'joint_1': (math.radians(30), 3, 0.0), 'joint_2': (math.radians(15), 5, 2.4),
          'joint_3': (math.radians(25), 4, 4.4)}
LOOP, RAMP = 120.0, 12.0


def run(ramp, seconds):
    """Step the ramp and return (levels, rates, accels) sampled every tick."""
    levels, rates, accels = [], [], []
    for _ in range(round(seconds / DT)):
        levels.append(ramp.step(DT))
        rates.append(ramp.rate)
        accels.append(ramp.accel)
    return levels, rates, accels


def settle(wander, seconds=None, tempo=1.0):
    """Advance a wander until its ramp has settled, or for the given seconds."""
    steps = round(seconds / DT) if seconds is not None else 10 ** 6
    for _ in range(steps):
        if seconds is None and wander.ramp.settled:
            return
        wander.advance(DT, tempo)


def test_a_ramp_starts_and_ends_at_rest():
    ramp = Ramp(10.0)
    ramp.aim(1.0)
    levels, rates, accels = run(ramp, 10.0)
    assert levels[-1] == 1.0 and rates[-1] == 0.0 and accels[-1] == 0.0
    assert ramp.settled
    assert abs(rates[0]) < 1e-3 and abs(accels[0]) < 1e-2
    assert all(later >= earlier for earlier, later in zip(levels, levels[1:]))


def test_a_ramp_takes_exactly_its_duration():
    ramp = Ramp(10.0)
    ramp.aim(1.0)
    levels, _, _ = run(ramp, 9.9)
    assert not ramp.settled and levels[-1] < 1.0
    run(ramp, 0.2)
    assert ramp.settled and ramp.level == 1.0


def test_a_ramp_peaks_at_the_minimum_jerk_rate():
    ramp = Ramp(10.0)
    ramp.aim(1.0)
    _, rates, _ = run(ramp, 10.0)
    assert max(rates) == pytest.approx(1.875 / 10.0, rel=1e-3)


def test_a_ramp_goes_back_down_the_same_way():
    ramp = Ramp(10.0)
    ramp.aim(1.0)
    run(ramp, 10.0)
    ramp.aim(0.0)
    levels, rates, _ = run(ramp, 10.0)
    assert levels[-1] == 0.0 and ramp.settled
    assert min(rates) == pytest.approx(-1.875 / 10.0, rel=1e-3)
    assert all(later <= earlier for earlier, later in zip(levels, levels[1:]))


@pytest.mark.parametrize('seconds', [0.5, 2.0, 5.0, 8.0, 11.0])
def test_a_ramp_reversed_part_way_has_no_jump_in_rate_or_acceleration(seconds):
    ramp = Ramp(12.0)
    ramp.aim(1.0)
    run(ramp, seconds)
    level, rate, accel = ramp.level, ramp.rate, ramp.accel
    ramp.aim(0.0)
    levels, rates, accels = run(ramp, 12.1)
    assert abs(levels[0] - level) <= abs(rate) * DT * 2 + 1e-6
    assert abs(rates[0] - rate) < 2e-3
    assert abs(accels[0] - accel) < 0.04
    # Every later tick changes the rate by a tiny amount, with no kink anywhere on the way.
    assert max(abs(later - earlier) for earlier, later in zip(rates, rates[1:])) < 2e-3
    assert min(levels) >= 0.0 and max(levels) <= 1.0
    assert levels[-1] == 0.0 and ramp.settled


def test_a_short_trip_takes_less_time_and_is_not_much_faster_or_sharper_than_a_full_one():
    full = Ramp(12.0)
    full.aim(1.0)
    _, full_rates, full_accels = run(full, 12.1)
    for seconds in (0.5, 2.0, 5.0, 11.9):
        ramp = Ramp(12.0)
        ramp.aim(1.0)
        run(ramp, seconds)
        ramp.aim(0.0)
        taken = 0.0
        rates, accels = [], []
        while not ramp.settled:
            ramp.step(DT)
            taken += DT
            rates.append(abs(ramp.rate))
            accels.append(abs(ramp.accel))
        assert taken <= 12.0 + DT
        assert max(rates) <= 1.3 * max(full_rates)
        assert max(accels) <= 3.6 * max(full_accels)


def test_stopping_straight_after_starting_is_over_within_a_moment():
    ramp = Ramp(12.0)
    ramp.aim(1.0)
    run(ramp, 0.3)
    ramp.aim(0.0)
    run(ramp, 1.5)
    assert ramp.settled and ramp.level == 0.0


def test_changing_the_goal_before_it_has_moved_at_all_is_settled_at_once():
    ramp = Ramp(12.0)
    ramp.aim(1.0)
    ramp.aim(0.0)
    run(ramp, 0.5)
    assert ramp.settled and ramp.level == 0.0


def test_a_ramp_given_the_goal_it_already_has_keeps_going():
    ramp = Ramp(10.0)
    ramp.aim(1.0)
    run(ramp, 3.0)
    level = ramp.level
    ramp.aim(1.0)
    run(ramp, 7.1)
    assert level < ramp.level == 1.0 and ramp.settled


def test_a_ramp_that_is_settled_stays_put():
    ramp = Ramp(10.0, level=1.0)
    assert ramp.settled
    assert ramp.step(DT) == 1.0 and ramp.rate == 0.0


@pytest.mark.parametrize('duration', [0.0, -1.0, math.inf, math.nan])
def test_a_ramp_needs_a_positive_finite_duration(duration):
    with pytest.raises(ValueError, match='duration'):
        Ramp(duration)


def test_the_swing_starts_exactly_at_the_centre_pose_and_without_moving():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    assert wander.positions() == CENTRE
    wander.advance(DT, 1.0)
    for joint in JOINTS:
        assert abs(wander.positions()[joint] - CENTRE[joint]) < 1e-6


def test_the_swing_grows_to_its_full_size_and_never_beyond():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    low, high = dict(CENTRE), dict(CENTRE)
    for _ in range(round(2 * LOOP / DT)):
        wander.advance(DT, 1.0)
        for joint, angle in wander.positions().items():
            low[joint], high[joint] = min(low[joint], angle), max(high[joint], angle)
    for joint, (amplitude, _, _) in SWINGS.items():
        assert high[joint] <= CENTRE[joint] + amplitude + 1e-9
        assert low[joint] >= CENTRE[joint] - amplitude - 1e-9
        # Over two loops each joint reaches nearly the whole of its swing.
        assert high[joint] - low[joint] > 1.9 * amplitude


def test_the_whole_motion_repeats_every_loop_once_it_has_grown():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    settle(wander)
    first = wander.positions()
    settle(wander, LOOP)
    again = wander.positions()
    for joint in JOINTS:
        assert again[joint] == pytest.approx(first[joint], abs=1e-9)


def test_the_motion_does_not_come_back_to_where_it_began_part_way_through_a_loop():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    settle(wander)
    start = wander.positions()
    for fraction in (0.5, 0.25):
        probe = Wander(CENTRE, SWINGS, LOOP, RAMP)
        settle(probe)
        settle(probe, LOOP * fraction)
        pose = probe.positions()
        assert max(abs(pose[joint] - start[joint]) for joint in JOINTS) > math.radians(5.0)


def test_stopping_returns_every_joint_to_the_centre_pose_exactly():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    settle(wander, 40.0)
    assert not wander.finished
    wander.ramp.aim(0.0)
    assert not wander.finished
    for _ in range(round((RAMP + 1.0) / DT)):
        wander.advance(DT, 1.0)
    assert wander.finished
    assert wander.positions() == CENTRE


def test_slowing_the_clock_slows_the_swing_and_the_start_alike():
    def peak(tempo, seconds):
        wander = Wander(CENTRE, SWINGS, LOOP, RAMP, tempo=tempo)
        before, fastest = wander.positions(), 0.0
        for _ in range(round(seconds / DT)):
            wander.advance(DT, tempo)
            now = wander.positions()
            fastest = max(fastest, max(abs(now[joint] - before[joint]) for joint in JOINTS) / DT)
            before = now
        return fastest, wander.ramp.settled

    full, full_settled = peak(1.0, 14.0)
    half, half_settled = peak(0.5, 14.0)
    assert full_settled and not half_settled
    assert half == pytest.approx(full / 2.0, rel=0.1)


def test_the_clock_changes_its_pace_gradually():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    paces = []
    for _ in range(round(6.0 / DT)):
        wander.advance(DT, 0.2)
        paces.append(wander.tempo)
    assert paces[0] == pytest.approx(1.0 - TEMPO_RATE * DT)
    assert all(abs(later - earlier) <= TEMPO_RATE * DT + 1e-12
               for earlier, later in zip(paces, paces[1:]))
    assert paces[-1] == pytest.approx(0.2)


def test_a_step_that_could_not_be_taken_is_undone():
    wander = Wander(CENTRE, SWINGS, LOOP, RAMP)
    settle(wander, 5.0)
    clock, tempo, level = wander.clock, wander.tempo, wander.ramp.level
    positions = wander.positions()
    wander.advance(DT, 0.5)
    assert wander.positions() != positions
    wander.rollback()
    assert (wander.clock, wander.tempo, wander.ramp.level) == (clock, tempo, level)
    assert wander.positions() == positions
    # There is only one step to undo.
    wander.rollback()
    assert wander.clock == clock


@pytest.mark.parametrize('loop', [0.0, -5.0, math.inf])
def test_a_wander_needs_a_positive_finite_loop(loop):
    with pytest.raises(ValueError, match='loop'):
        Wander(CENTRE, SWINGS, loop, RAMP)
