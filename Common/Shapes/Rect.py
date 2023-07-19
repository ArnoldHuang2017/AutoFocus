# -- coding: utf-8 -

class Rect(object):
    def __init__(self, left, top, width, height):
        self._shape = [left, top, width, height]

    def tl(self):
        return self._shape[0], self._shape[1]

    def br(self):
        return self._shape[0] + self._shape[2], self._shape[1] - self._shape[3]

