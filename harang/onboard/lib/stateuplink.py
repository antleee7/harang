# State Uplink

# 지상국 --> avionics. 상대적으로 단순합니다.

class State:

    def __init__(self):

        # Define states and their bit positions in a byte
        self.IDLE          = 0  # default, no bits high

        # Stand-by mode에서 Launch Ready Mode로의 Transition명령 비트입니다.
        self.LAUNCH           = 1
        self.LAUNCH_BIT       = 0

        # 강제사출 명령 비트입니다.
        self.DEPLOY       = 2
        self.DEPLOY_BIT   = 1 

        # Define state value holder
        self.state = self.IDLE

    # Set state
    def set(self, new_state):
        self.state = new_state

    # Add state (input each state as separate parameter)
    def add(self, *new_states):
        for new_state in new_states:
            self.state += new_state

    # Remove state
    def remove(self, *new_states):
        for new_state in new_states:
            self.state -= new_state

    # Retrieve value of bit at index in state. Return True if bit is 1. If a
    # value for byte is specified, the bit is checked in that, not in state.
    def get_bit(self, idx, byte=None):
        if byte is None:
            return ((self.state & (1 << idx)) != 0);
        else:
            return ((byte & (1 << idx)) != 0);

    # Return state as string
    def __str__(self):
        # If we are idle, return
        if self.state == self.IDLE:
            return "Idle"
        # Otherwise, parse out the flags
        ret = ""
        if self.get_bit(self.LAUNCH_BIT):       ret += "Launch ready mode transition order transmitted + "
        if self.get_bit(self.DEPLOY_BIT):       ret += "Drogue deploy order transmitted + "
        return ret[:-3]  # remove trailing ' + '