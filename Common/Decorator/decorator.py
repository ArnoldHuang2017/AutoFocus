from functools import wraps


def singleton(cls):
    instances = {}

    @wraps(cls)
    def getinstance(*args, **kwages):
        if cls not in instances:
            instances[cls] = cls(*args, **kwages)

        return instances[cls]

    return getinstance
