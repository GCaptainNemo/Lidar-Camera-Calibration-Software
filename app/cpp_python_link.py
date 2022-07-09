import ctypes

clib = ctypes.CDLL("../build/devel/lib/libLidarCameraFusion.so")

class InitialParametersStruct(ctypes.Structure):
    _fields_ = [  
                 ("calib_file_address", ctypes.c_char_p), 
                 ("lidar_address", ctypes.c_char_p), 
                 ("lidar_type", ctypes.c_char_p), 
                 ("img_address", ctypes.c_char_p),
                 ] 

class CalibrationParametersStruct(ctypes.Structure):  
    _fields_ = [ ("fx", ctypes.c_float), ("fy", ctypes.c_float), ("cx", ctypes.c_float), ("cy", ctypes.c_float), 
                 ("k1", ctypes.c_float), ("k2", ctypes.c_float), ("p1", ctypes.c_float), ("p2", ctypes.c_float), 
                 ("tx", ctypes.c_float), ("ty", ctypes.c_float), ("tz", ctypes.c_float), 
                 ("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float), 
                 ] 

class StructImage(ctypes.Structure):  
    _fields_ = [ ("img", ctypes.c_ubyte * 6220800), ("row", ctypes.c_int), ("col", ctypes.c_int), ("channel", ctypes.c_int)]  

