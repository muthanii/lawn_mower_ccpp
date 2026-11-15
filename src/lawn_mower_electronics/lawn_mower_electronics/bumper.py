"""Bumper: a contact strip round the body.

The simulator reports the contacts of the chassis while it touches something.
The bumper is pressed from a report until hold_time after the last one, so a
single missed report does not release it.
"""


class Bumper:

    def __init__(self, hold_time):
        self.hold_time = hold_time
        self.last_contact = None

    def touch(self, t):
        """A contact was reported at time t [s]."""
        self.last_contact = t

    def pressed(self, t):
        return self.last_contact is not None and t - self.last_contact <= self.hold_time
