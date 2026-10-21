"""Internal: phase-callback dispatch for `Robot.run()`.

Owns the run-time plumbing so the student's code is just three callbacks.

Sequence (see spec sec.3):
  1. arm() on entry (after link-up + config verify)
  2. settle for STATUS_SETTLE, then call initialize(robot) once, then idle the
     setpoints - initialize() is for setup, not motion, and the extension drops
     commands outside AUTON/DRIVER
  3. loop: when the brain reports a new phase, dispatch the matching callback
     (autonomous / driver / ... ) on its own daemon thread; the callback owns
     its own `while robot.running:` loop
  4. when a callback returns BEFORE its phase ends, zero the setpoint buffer
     (IDLE on all effector ports). TX loop keeps emitting keepalives so the
     brain's signal-loss failsafe does not fire spuriously. The callback is NOT
     re-invoked for that phase.
  5. disarm() on exit. Block until session/match ends.

A callback that ignores `robot.running` and runs past the end of its phase gets
an exception injected into its thread so it unwinds (prints and all). That
lands at the next bytecode, so one blocked in a long sleep or a C call keeps
going until it returns - which is why its setpoint ownership is revoked too,
not instead: either way the robot idles on time.
"""

from __future__ import annotations

import ctypes
import inspect
import threading
import time
from typing import Callable, TYPE_CHECKING

from .enums import Phase

if TYPE_CHECKING:
    from .robot import Robot


TICK = 0.02  # 50 Hz: the rate the TX loop emits command frames at.
STOP_GRACE = 0.25  # how long to let an overrunning callback unwind before reporting it
STATUS_SETTLE = 0.02  # how long to let the brain's first telemetry land before initialize()


class _PhaseEnded(BaseException):
    """Injected into a callback thread that outran its phase, to unwind it.

    BaseException so a student's `except Exception:` does not swallow the stop.
    """


# Map from match phase -> the kwarg name on Robot.run() that supplies its callback.
_PHASE_TO_CALLBACK_KEY: dict[Phase, str] = {
    Phase.AUTON: "autonomous",
    Phase.DRIVER: "driver",
    # PAUSE / POSTMATCH intentionally have no student callback in v1;
    # the runtime simply idles (zero setpoints + keepalive) during those phases.
}


def run_match(
    robot: "Robot",
    initialize: Callable[["Robot"], None] | None,
    autonomous: Callable[["Robot"], None] | None,
    driver: Callable[["Robot"], None] | None,
) -> None:
    """Drive the full match: arm, dispatch phase callbacks, disarm.

    Blocks until the session/match ends. See module docstring for sequence.
    """
    callbacks = {"autonomous": autonomous, "driver": driver}
    last_phase: Phase | None = None
    worker: threading.Thread | None = None

    for name, callback in callbacks.items():
        _warn_if_loopless(name, callback)

    robot.arm()

    try:
        if initialize is not None:
            # The extension pushes a placeholder status (battery_v 0) the moment
            # the IPC client connects, and the brain's own telemetry lands a
            # little later. Without this, initialize() reads robot.battery_voltage
            # as 0.0 and prints a number that is not the battery's.
            time.sleep(STATUS_SETTLE)
            initialize(robot)
            # initialize() runs on the runtime thread before any phase has claimed
            # the setpoints, so its effector writes are accepted into the buffer.
            # Drop them: a normal start would zero them on the pause-phase branch
            # below anyway, but a mid-match re-run enters straight into AUTON and
            # would otherwise flush initialize's setpoints the moment the extension
            # starts forwarding commands - movement no callback ever asked for.
            robot._state.zero_setpoints()

        while True:
            current = robot.phase
            if current in {Phase.POSTMATCH, Phase.ESTOP}:
                break

            if current == last_phase:
                time.sleep(TICK)
                continue

            last_phase = current
            key = _PHASE_TO_CALLBACK_KEY.get(current)
            callback = callbacks.get(key) if key else None
            if callback is None:
                # A phase with no callback is legal (a driver-only program is a
                # normal thing to write), but silence here reads exactly like a
                # program that failed to start, so say which kwarg is missing.
                # PAUSE/POSTMATCH have no key at all and pass quietly.
                if key:
                    print(
                        f"[API] the {current.value} phase started, but run() was not given "
                        f"{key}=; the robot idles until the phase changes."
                    )
                robot._state.zero_setpoints()
                continue

            robot._runtime_active_phase = current
            robot._runtime_callback_active = True
            failure: list[BaseException] = []
            worker = threading.Thread(
                target=_invoke,
                args=(robot, callback, failure),
                name=f"marlin-{current.value}",
                daemon=True,
            )
            worker.start()
            while worker.is_alive() and robot.phase == current:
                time.sleep(TICK)

            # Phase over (or callback done): drop `robot.running`, take setpoint
            # ownership back off the callback, and idle the effectors. A callback
            # still executing at this point can no longer drive anything.
            robot._runtime_callback_active = False
            robot._state.claim_setpoints()
            robot._state.zero_setpoints()

            overran = _stop_thread(worker)

            if failure:
                raise failure[0]
            if worker.is_alive():
                print(
                    f"[API] {key}() is still running after the {current.value} phase "
                    f"ended; its outputs are ignored."
                )
            elif overran:
                print(
                    f"[API] {key}() ran past the end of the {current.value} phase; "
                    f"MARLIN stopped it."
                )
            elif robot.phase == current:
                print(
                    f"[API] {key}() returned before the {current.value} phase ended; "
                    f"the robot idles until the phase changes."
                )

            while robot.phase == current and current not in {Phase.POSTMATCH, Phase.ESTOP}:
                time.sleep(TICK)
    finally:
        robot._runtime_callback_active = False
        robot._runtime_active_phase = None
        if worker is not None:
            _stop_thread(worker)  # ESTOP / Ctrl-C / raised failure: leave nothing looping
        robot._state.zero_setpoints()
        robot.disarm()


def _invoke(robot: "Robot", callback: Callable[["Robot"], None], failure: list) -> None:
    """Run one phase callback on its own thread; only its writes are accepted."""
    robot._state.claim_setpoints()
    try:
        callback(robot)
    except BaseException as exc:  # surfaced on the runtime thread, which owns disarm()
        if not isinstance(exc, _PhaseEnded):
            failure.append(exc)


def _stop_thread(worker: threading.Thread) -> bool:
    """Unwind an overrunning callback thread. True if it was still going.

    ponytail: CPython's async-exc hook is the only way to interrupt a thread
    that never checks `robot.running`. Ceiling: it fires at the next bytecode,
    so a callback parked in `time.sleep(30)` or a blocking read keeps its own
    prints until that call returns - nothing short of a subprocess fixes that.
    """
    ident = worker.ident
    if not worker.is_alive() or ident is None:
        return False

    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_ulong(ident), ctypes.py_object(_PhaseEnded)
    )
    worker.join(STOP_GRACE)
    return True


def _warn_if_loopless(name: str, callback: Callable[["Robot"], None] | None) -> None:
    """Warn at startup about the two shapes that misbehave: no loop, or an unbounded one.

    ponytail: substring scan, not an AST walk, and it matches `running` rather
    than `robot.running` because the parameter is not always named `robot`. A
    loop hidden in a helper the callback calls reads as loopless here, so this
    only ever warns - the runtime reports the same two mistakes after the fact.
    """
    if callback is None:
        return
    try:
        source = inspect.getsource(callback)
    except (OSError, TypeError):
        return  # lambda in a REPL, C-defined, source unavailable - not worth chasing

    if "while " not in source and "for " not in source:
        print(
            f"[API] {name}() has no loop. MARLIN calls it once when the phase "
            f"starts, so it needs `while robot.running:` to keep control."
        )
    elif "running" not in source:
        print(
            f"[API] {name}() loops but never checks robot.running, so it will not "
            f"stop when the phase ends. MARLIN stops it there instead."
        )


def is_running(robot: "Robot") -> bool:
    """True while the current phase is an active run phase AND its callback is still in scope.

    `robot.running` is scoped per-phase: it flips False when that phase ends, so
    the callback's `while robot.running` loop exits naturally. The PAUSE between
    auton and driver is a genuine non-running interval (not a hack).

    Reading it also paces the caller to 50 Hz, so `while robot.running:` runs at
    the TX rate whether or not the student wrote `time.sleep(0.02)`.
    """
    # ponytail: a blocking property read is the cheapest way to stop a sleepless
    # student loop from pegging a core and starving the RX thread. Pace first,
    # then sample the phase, so the answer is fresh when it is returned.
    # Ceiling: two `robot.running` reads per iteration halve the rate - add an
    # explicit robot.tick() if that ever matters.
    time.sleep(max(0.0, TICK - (time.monotonic() - robot._last_tick)))
    robot._last_tick = time.monotonic()

    phase = robot.phase
    return bool(
        phase in _PHASE_TO_CALLBACK_KEY
        and getattr(robot, "_runtime_callback_active", False)
        and getattr(robot, "_runtime_active_phase", None) == phase
    )
