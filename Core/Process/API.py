from .Main import *


def Start(cal, len_number=None, name=None, closest=None):
    MainProcess().MainProcess(cal, len_number,  name, closest)


__all__ = ["Start"]