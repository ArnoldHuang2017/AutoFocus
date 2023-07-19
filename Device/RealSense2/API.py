from .D435Camera import D435Camera, RS2_WIDTH, RS2_HEIGHT, RS2_FPS, RS2_WIDTH_COMPENSATE, RS2_HEIGHT_COMPENSATE


def OpenCamera():
    return D435Camera().Open()


def CloseCamera():
    return D435Camera().Close()


def ReadCamera():
    return D435Camera().Read()


__all__ = ["OpenCamera", "ReadCamera", "CloseCamera", "RS2_WIDTH", "RS2_HEIGHT", "RS2_FPS", "RS2_WIDTH_COMPENSATE",
           "RS2_HEIGHT_COMPENSATE"]
