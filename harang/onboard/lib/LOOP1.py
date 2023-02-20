# LOOP1

# Sensor raw data 기록 담당입니다.

# --INFLIGHT DATA PACKET--
# 1 data type bit
# EBIMU-9DOFv5: ax, ay, az, gx, gy, gz, mx, my, mz (int_16)
# BMP390: temp(int_8), alt(int_16)
# SYSTEM: time(int_32), status, parachute, proximity sensor
# 1 error correction bit

# --PREFLIGHT DATA PACKET--
# 1 data type bit
# ?: volt(int_8)
# EBIMU-9DOFv5: ax, ay, az (int_16)
# BMP390: temp(int_8)
# 1 STATUS bit
# 1 SD_STATUS bit
# 1 IMU_STATUS bit

import os
import time
from picamera import PiCamera
from datetime import datetime

# Define directories (include terminating forward-slash!)
ROOT      = "./"
LOG_DIR   = ROOT + "log/"
VIDEO_DIR = ROOT + "video/"
DEBUG_DIR = ROOT + "debug/"
EXT       = "csv"
VIDEO_EXT = "h264" # .h264 코덱 사용. 실제로 드론 영상에서 많이 쓰이는 코덱임

# Define constants for Logger write targets.
LOG   = 0
DEBUG = 1
ALL   = 2

# Video-specific parameters
CAPTURE_RES = (1920, 1080)  # in pixels



class Logger:

    def __init__(self, init_log=True, init_camera=True, init_debug=True):
        self.log            = None  # Data log file descriptor
        self.logfilename    = None
        self.log_enabled    = False
        self.debug          = None  # Debug file descriptor
        self.debugfilename  = None
        self.debug_enabled  = False
        self.camera         = None
        self.camera_enabled = False
        if init_log:    self.log_enabled    = self._init_new_log()
        if init_debug:  self.debug_enabled  = self._init_new_debug()
        if init_camera: self.camera_enabled = self._init_camera()

    def start_video(self):
        if self.camera_enabled: 
            self.camera.start_recording(VIDEO_DIR + self.logfilename + "." + VIDEO_EXT)

    def stop_video(self):
        if self.camera_enabled:
            self.camera.stop_recording()

    # Write data to file. Data is specified as a list, and delimeter is used
    # to separate each data point when written. If flush is True, this function
    # immediately flushes the written data to file as well.
    #
    # Target can be either LOG (0) or DEBUG (1). By default, target is LOG file.
    def write(self, data, target=LOG, delimeter=",", flush=True):
        # Write to file and flush (if specified)
        if target == LOG:
            # Append each element in data list to a string
            output = ""
            n      = len(data)
            for idx, val in enumerate(data):
                output += ("%s" % val)
                # Add delimeter or newline to end of string
                if idx == (n-1):
                    output += "\n"
                else:
                    output += delimeter + "\t"
            self.log.write(output)
            if flush: self.log.flush()
        elif target == DEBUG:
            # Print debug message with timestamp
            output = "[" + datetime.now().strftime("%T") + "] " + data + "\n"
            self.debug.write(output)
            if flush: self.debug.flush()
        else:
            print("INVALID TARGET SPECIFIED")
            return False
        return True  # success

    # Stop logger. This consists of log file, debug file, and video capture, if
    # they are enabled.
    def stop(self, target=LOG):
        if (target == ALL or target == LOG) and self.log_enabled:
            self.log.flush()
            self.log.close()
        if (target == ALL or target == DEBUG) and self.debug_enabled:
            self.debug.flush()
            self.debug.close()
        if (target == ALL or target == LOG) and self.camera_enabled:
            self.stop_video()

    # Open file descriptor to new log file
    def _init_new_log(self):
        self.logfilename = self.__generate_filename(LOG_DIR);
        self.log = open("%s%s.%s" % (LOG_DIR, self.logfilename, EXT), "a")
        print "CREATED LOG FILE (%s.%s)" % (self.logfilename, EXT)
        return True

    # Open file descriptor to new debug file
    def _init_new_debug(self):
        self.debugfilename = self.__generate_filename(DEBUG_DIR);
        self.debug = open("%s%s.%s" % (DEBUG_DIR, self.debugfilename, EXT), "a")
        print "CREATED DEBUG FILE (%s.%s)" % (self.debugfilename, EXT)
        return True

    # Configure camera on the Pi
    def _init_camera(self):
        try:
            self.camera = PiCamera()
            self.camera.resolution = CAPTURE_RES
            print("INITIALIZED CAMERA TO %d*%d px" % CAPTURE_RES)
            return True
        except:
            print("FAILED TO INITIALIZE CAMERA")
            return False

    # Generate new filename based on what already exists in specified directory
    def __generate_filename(self, directory, extension=EXT):
        # For every file in the log directory ending with EXT extension, we 
        # parse the number at the start of the filename, and if it is larger
        # than the number we have saved in last_launch, we update last_launch
        # to this new value.
        last_launch = 0
        for f in [x for x in os.listdir(directory) if x.endswith(extension)]:
            try:
                launch_numb = int(f.split("_")[0])  # get number at start of file
                last_launch = launch_numb if launch_numb > last_launch else last_launch
            except ValueError:
                print "WARNING: INVALID .%s FILE IN LOGS FOLDER (%s)" % (extension, f)
                return False

        # Next, we generate a new filename based of the current system time.
        # The now() function returns the system time, and on the Raspberry Pi, 
        # without internet connection, this function will return the date from
        # last internet contact (i.e. this date will be wrong most of the 
        # time unless you connect to WiFi).
        date          = datetime.now().strftime("%y-%m-%d")
        new_file_name = "%03d_%s" % (last_launch + 1, date)

        # Generate the full save path and verify that this file does not 
        # already exist.
        fullpath = "%s%s.%s" % (directory, new_file_name, extension)
        if os.path.exists(fullpath):
            print "ERROR: LOG FILE ALREADY EXISTS (%s)" % fullpath
            # TODO: make this error more visible
            return False
        else:
            return new_file_name
