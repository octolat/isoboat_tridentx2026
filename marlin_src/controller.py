"""Controller, Joystick, and Button - sourced from the C3's own USB packets.

Controller state is independent of the BLE/robot link: the C3 injects its own
controller-input packets over USB, so `robot.controller` works even when the
robot link is down.

The panel carries four single-axis joysticks, numbered ONE..FOUR on the
hardware: ONE and TWO on the left, THREE and FOUR on the right. Those numbers
are the only names students use - there are no aliases.
"""

from __future__ import annotations


# Panel number -> the axis key the C3's controller packet carries it under.
# Controller-side code (the extension's decoder, the calibration file, the IPC
# payload) speaks the left/right stick convention; everything student-facing
# speaks the panel numbers. This dict is the one place the two meet.
JOYSTICK_AXES = {
    "ONE": "left_stick_x",
    "TWO": "left_stick_y",
    "THREE": "right_stick_x",
    "FOUR": "right_stick_y",
}
JOYSTICK_NAMES = tuple(JOYSTICK_AXES)

# The eight mask bits the firmware names (MARLINCEP v1.1 byte 9) plus the two
# triggers the codec folds in from byte 10. Documented spelling; lookups are
# case-insensitive.
BUTTON_NAMES = (
    "UP",
    "DOWN",
    "LEFT",
    "RIGHT",
    "X",
    "Y",
    "A",
    "B",
    "LEFT_TRIGGER",
    "RIGHT_TRIGGER",
)
# Lookups uppercase the requested name, and StateMirror uppercases every key it
# caches. Every name above is already uppercase, so this is a no-op today - kept
# so a lower-case name added later cannot silently stop matching.
_BUTTON_KEYS = {name.upper() for name in BUTTON_NAMES}


def _controller_state(state_mirror) -> dict:
    if state_mirror is None:
        return {}
    return state_mirror.get_controller() or {}


class Joystick:
    """
    One joystick on the panel. It moves along a single axis, not two.

    `value` is the one to use: -100 to 100, and 0 when the stick is centred.
    """

    def __init__(self, name: str, axis: str, state_mirror=None) -> None:
        """Bind this joystick to its panel number, its packet axis, and the state mirror."""
        self.name: str = name
        self._axis = axis
        self._state = state_mirror

    @property
    def value(self) -> float:
        """
        How far the stick is pushed: -100 to 100, and 0 when centred.

        The sign says which way it is pushed. These numbers line up with a
        thruster's duty range, so a stick can drive a thruster directly.
        """
        return float(_controller_state(self._state).get(f"{self._axis}_scaled", 0.0))

    @property
    def raw(self) -> int:
        """
        The unscaled electrical reading behind `value`: 0..4095, and about
        2048 when centred.

        Use `value` unless you have a reason not to - `raw` is here for
        checking a stick's true centre or writing your own scaling.
        """
        return int(_controller_state(self._state).get(f"{self._axis}_raw", 2048))


class Button:
    """
    One button on the panel. It only tells you if it is down right now.

    There is deliberately no "was just pressed" helper. Spotting the moment a
    button goes from up to down means remembering what it read last time round
    your loop, and working that out is part of the point.

    Do NOT add `pressed`/edge helpers here.
    """

    def __init__(self, name: str, state_mirror=None) -> None:
        """Bind this button to its hardware name and the state mirror."""
        self._name = name
        self._state = state_mirror

    @property
    def is_down(self) -> bool:
        """True while the button is held down, False as soon as it is let go.

        It stays True for every pass of your loop that the button is held, so
        it tells you the button is down now - not that a new press happened.
        """
        buttons = _controller_state(self._state).get("buttons", {})
        return bool(buttons.get(self._name, False))


class Controller:
    """
    The handheld control panel. You reach it as `robot.controller`.

        forward = robot.controller.joystick("ONE").value
        if robot.controller.button("A").is_down:
            ...

    The panel is wired to the C3 box, so reading it still works even when the
    wireless link to the robot is down.
    """

    def __init__(self, state_mirror=None) -> None:
        """Construct the four joysticks and a button accessor backed by the state mirror."""
        self._state = state_mirror
        self._joysticks: dict[str, Joystick] = {
            name: Joystick(name, axis, state_mirror) for name, axis in JOYSTICK_AXES.items()
        }
        self._buttons: dict[str, Button] = {}

    def joystick(self, name: str) -> Joystick:
        """
        Return one of the four joysticks, named by the number on the panel.

        Use the number printed next to the stick: "ONE", "TWO", "THREE" or
        "FOUR". These are the only names; capitalisation does not matter.

            forward = robot.controller.joystick("ONE").value

        Any other name raises ValueError rather than handing back a stick that
        would silently read 0 forever.
        """
        key = str(name).upper()
        if key not in self._joysticks:
            raise ValueError(
                f"Unknown joystick {name!r}. Expected one of {', '.join(JOYSTICK_NAMES)}."
            )
        return self._joysticks[key]

    def button(self, name: str) -> Button:
        """
        Return a button by the name printed on the panel.

        One of "UP", "DOWN", "LEFT", "RIGHT", "X", "Y", "A", "B",
        "LEFT_TRIGGER", "RIGHT_TRIGGER". These are the only names, though
        capitalisation does not matter.

            if robot.controller.button("A").is_down:
                ...

        An unknown name raises ValueError; without that, a typo would just read
        False forever and look like a broken button.
        """
        key = str(name).upper()
        if key not in _BUTTON_KEYS:
            raise ValueError(
                f"Unknown button {name!r}. Expected one of {', '.join(BUTTON_NAMES)}."
            )
        if key not in self._buttons:
            self._buttons[key] = Button(key, self._state)
        return self._buttons[key]
