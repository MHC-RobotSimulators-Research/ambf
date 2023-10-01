import math
import time


class ambf_xbox_controller_fake:

    def __init__(self):
        self.counter = 1

    def read(self):
        # print(self.counter)
        if 1000 <= self.counter:
            self.counter = 1

        lx = math.cos(2 * math.pi * (1000 / self.counter))
        ly = math.sin(2 * math.pi * (1000 / self.counter))
        lt = math.cos(math.pi * (1000 / self.counter))
        lb = 0

        rx = math.cos(2 * math.pi * (1000 / self.counter))
        ry = math.sin(2 * math.pi * (1000 / self.counter))
        rt = math.cos(math.pi * (1000 / self.counter))
        rb = 0

        self.counter += 1

        return [[lx, ly, lt, lb], [rx, ry, rt, rb], [0, 0, 0, 0, 0, 0]]

    def rumble(self):
        pass
