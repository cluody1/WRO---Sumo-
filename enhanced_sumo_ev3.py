#!/usr/bin/env python3
# enhanced_sumo_ev3.py
# ev3dev2-based Sumo robot controller (enhanced)
# Features:
# - Dual ultrasonic aiming (left/right) with fallback to single front US
# - Reflectance (color) edge detection (left/right/back)
# - PID steering for approach (uses US left/right difference)
# - State machine: SEARCH -> LOCKON -> APPROACH -> PUSH -> RECOVER
# - CSV logging for tests (timestamps, sensors, errors, PID output, state)
# - Edge detection is highest priority and triggers immediate recovery
# Wiring (example): adjust ports to your setup
#   LEFT_MOTOR = OUTPUT_B, RIGHT_MOTOR = OUTPUT_C
#   US_LEFT = INPUT_3, US_RIGHT = INPUT_4, COLOR_LEFT=INPUT_1, COLOR_RIGHT=INPUT_2
# Requires: python-ev3dev2 on the EV3 Brick

from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from time import sleep, time
import csv, os

# --- CONFIG --- (adjust to your wiring)
LEFT_MOTOR_PORT  = OUTPUT_B
RIGHT_MOTOR_PORT = OUTPUT_C
US_LEFT_PORT  = INPUT_3
US_RIGHT_PORT = INPUT_4
COLOR_LEFT_PORT  = INPUT_1
COLOR_RIGHT_PORT = INPUT_2
BUMPER_PORT = None  # set to INPUT_4 if using touch sensor

# Movement speeds (percent)
BASE_SPEED_SEARCH = 20
BASE_SPEED_APPROACH = 30
BASE_SPEED_PUSH = 85

# Edge thresholds (reflectance 0-100)
REFLECT_WHITE_THRESHOLD = 40

# Ultrasonic thresholds (cm)
US_DETECT_CM = 25    # detect opponent within ~25 cm
US_PUSH_CM = 7       # start pushing when very close (~7 cm)

# PID params
KP = 1.2
KI = 0.01
KD = 0.08
PID_LIMIT = 40  # percent

# Timing
PUSH_MAX_TIME = 2.0
EDGE_BACKUP_TIME = 0.5
EDGE_ROTATE_TIME = 0.6

# Logging
LOG_FOLDER = "/home/robot/logs"
LOG_FILENAME = "sumo_test_log.csv"

# --- SETUP HARDWARE ---
tank = MoveTank(LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)
us_left = UltrasonicSensor(US_LEFT_PORT)
us_right = UltrasonicSensor(US_RIGHT_PORT)
cs_left = ColorSensor(COLOR_LEFT_PORT)
cs_right = ColorSensor(COLOR_RIGHT_PORT)

# --- UTILITIES ---
def ensure_log_folder():
    try:
        os.makedirs(LOG_FOLDER, exist_ok=True)
    except Exception:
        pass

def log_row(row):
    ensure_log_folder()
    path = os.path.join(LOG_FOLDER, LOG_FILENAME)
    header = ['timestamp','state','us_left_cm','us_right_cm','cs_left','cs_right','error','pid_out','left_speed','right_speed']
    write_header = not os.path.exists(path)
    try:
        with open(path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(header)
            writer.writerow(row)
    except Exception as e:
        print("Logging failed:", e)

def read_reflectance(sensor):
    try:
        return sensor.reflected_light_intensity
    except:
        return sensor.value()

def edge_detected():
    l = read_reflectance(cs_left)
    r = read_reflectance(cs_right)
    return (l >= REFLECT_WHITE_THRESHOLD) or (r >= REFLECT_WHITE_THRESHOLD)

def us_distance_cm(sensor):
    try:
        return sensor.distance_centimeters
    except:
        return None

# --- PID class ---
class PID:
    def __init__(self,kp,ki,kd,limit=None):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.limit = limit
        self._prev = 0.0; self._int = 0.0; self._last = None
    def reset(self):
        self._prev = 0.0; self._int = 0.0; self._last = None
    def compute(self,error):
        now = time()
        dt = 0.01 if self._last is None else max(0.001, now - self._last)
        self._last = now
        self._int += error * dt
        derivative = (error - self._prev) / dt if dt>0 else 0.0
        self._prev = error
        out = self.kp*error + self.ki*self._int + self.kd*derivative
        if self.limit is not None:
            if out>self.limit: out=self.limit
            if out<-self.limit: out=-self.limit
        return out

pid = PID(KP,KI,KD,PID_LIMIT)

# --- STATES ---
STATE_SEARCH = "SEARCH"
STATE_LOCKON = "LOCKON"
STATE_APPROACH = "APPROACH"
STATE_PUSH = "PUSH"
STATE_RECOVER = "RECOVER"

def stop():
    tank.stop()

def drive(base,left_correction):
    left = base - left_correction
    right = base + left_correction
    left = max(-100,min(100,left)); right = max(-100,min(100,right))
    tank.on(SpeedPercent(left), SpeedPercent(right))
    return left, right

def reverse(sec=-EDGE_BACKUP_TIME,power= -30):
    tank.on_for_seconds(SpeedPercent(power), SpeedPercent(power), sec, block=True)
    tank.stop()

def rotate(left_speed=30,right_speed=-30,seconds=EDGE_ROTATE_TIME):
    tank.on_for_seconds(SpeedPercent(left_speed), SpeedPercent(right_speed), seconds, block=True)
    tank.stop()

def main_loop():
    state = STATE_SEARCH
    pid.reset()
    push_start = None
    print("Enhanced Sumo starting")
    while True:
        # --- EDGE PRIORITY ---
        if edge_detected():
            print("EDGE! recovering...")
            stop()
            reverse()
            # determine which side saw edge and rotate away
            l = read_reflectance(cs_left); r = read_reflectance(cs_right)
            if l>r:
                rotate(left_speed=-30,right_speed=30,seconds=EDGE_ROTATE_TIME)
            else:
                rotate(left_speed=30,right_speed=-30,seconds=EDGE_ROTATE_TIME)
            state = STATE_SEARCH
            pid.reset()
            continue

        us_l = us_distance_cm(us_left)
        us_r = us_distance_cm(us_right)
        us_center = None
        if us_l and us_r:
            # use average as center distance
            us_center = (us_l + us_r) / 2.0

        # Logging prep variables
        left_speed_cmd = 0; right_speed_cmd = 0; pid_out = 0; error = 0

        if state == STATE_SEARCH:
            # gentle spin to find opponent
            tank.on(SpeedPercent(BASE_SPEED_SEARCH), SpeedPercent(-BASE_SPEED_SEARCH))
            if (us_l and us_l < US_DETECT_CM) or (us_r and us_r < US_DETECT_CM):
                stop(); state = STATE_LOCKON; pid.reset(); sleep(0.05); continue

        elif state == STATE_LOCKON:
            # align orientation to opponent: simple approach - rotate toward the closer US
            if us_l and us_r:
                if abs(us_l - us_r) < 2.0:
                    state = STATE_APPROACH
                elif us_l < us_r:
                    # opponent is more left -> rotate left slowly
                    tank.on(SpeedPercent(15), SpeedPercent(-15))
                else:
                    tank.on(SpeedPercent(-15), SpeedPercent(15))
            else:
                # fallback: short forward then search again for better reading
                tank.on_for_seconds(SpeedPercent(15), SpeedPercent(15), 0.25, block=True)
                state = STATE_SEARCH

        elif state == STATE_APPROACH:
            # compute steering error from US difference (right - left positive -> target to right)
            if us_l and us_r:
                error = (us_r - us_l) / max(1.0, (us_r + us_l))  # normalized small range
                pid_out = pid.compute(error)
                left_speed_cmd, right_speed_cmd = drive(BASE_SPEED_APPROACH, pid_out)
                # transition to push if very close or bumper pressed
                if (us_center and us_center <= US_PUSH_CM) or (has_bumper and bumper.is_pressed):
                    state = STATE_PUSH; push_start = time(); print("Entering PUSH")
            else:
                # Lost precise readings: slow down and search
                stop(); state = STATE_SEARCH; pid.reset()

        elif state == STATE_PUSH:
            # full force push - monitor time and edge
            tank.on(SpeedPercent(BASE_SPEED_PUSH), SpeedPercent(BASE_SPEED_PUSH))
            if push_start and (time() - push_start > PUSH_MAX_TIME):
                print("Push timeout - back off")
                stop(); reverse(sec=0.4,power=-40); rotate(); state = STATE_SEARCH; pid.reset()

        # Write log row each loop iteration
        timestamp = datetime.utcnow().isoformat()
        log_row([timestamp,state,us_l,us_r,read_reflectance(cs_left),read_reflectance(cs_right),error,pid_out,left_speed_cmd,right_speed_cmd])
        sleep(0.02)

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        stop()
        print("Stopped by user")
