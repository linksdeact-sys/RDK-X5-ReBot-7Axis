"""Read actual SDK receive timestamps, not the MotorBridge state cache age."""
import ctypes
import os


class RxFreshness:
    def __init__(self):
        path = os.environ.get("MOTOR_DM_DEVICE_LIB", "")
        self.library = ctypes.CDLL(path)
        self.read = self.library.rebot_rx_meta
        self.read.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_uint64)]
        self.read.restype = ctypes.c_int

    def snapshot(self):
        result = []
        for i in range(7):
            age, seq = ctypes.c_double(), ctypes.c_uint64()
            if self.read(0x11+i, ctypes.byref(age), ctypes.byref(seq)):
                raise RuntimeError("RX时间戳读取失败")
            result.append({"age_ms": age.value*1000, "sequence": seq.value})
        return result
