"""DuckBot navigation configuration — tune these values for your setup."""

LEFT_PIN = 12
RIGHT_PIN = 13

PERIMETER_WIDTH_CM = 1000
PERIMETER_HEIGHT_CM = 200
PERIMETER_CM = [
    (0, 0),
    (PERIMETER_WIDTH_CM, 0),
    (PERIMETER_WIDTH_CM, PERIMETER_HEIGHT_CM),
    (0, PERIMETER_HEIGHT_CM),
]

START_X_CM = 500
START_Y_CM = 100
START_HEADING_RAD = 0.0

END_X_CM = 900
END_Y_CM = 100

SONAR_TRIG = 23
SONAR_ECHO = 24

SONAR_BLIND_ZONE_CM = 20

def compute_obstacle_threshold(cfg):
    reaction_cm = (cfg["EXPLORE_SPEED"] * cfg["MAX_SPEED_CM_S"]) / cfg["LOOP_HZ"]
    reverse_cm = cfg["AVOID_REVERSE_SPEED"] * cfg["MAX_SPEED_CM_S"] * cfg["AVOID_REVERSE_TIME"]
    distance_cm = SONAR_BLIND_ZONE_CM + 5 + reaction_cm + reverse_cm
    return max(30, round(distance_cm))

BRAIN_CFG = {
    "PERIMETER_MARGIN_CM": 30,
    "EXPLORE_SPEED": 0.4,
    "TURN_SPEED": 0.5,
    "AVOID_REVERSE_SPEED": 0.5,
    "SCAN_SPEED": 0.4,
    "COMPLETE_TURN_SPEED": 0.5,
    "AVOID_REVERSE_TIME": 0.5,
    "SCAN_LEFT_TIME": 0.6,
    "SCAN_RIGHT_TIME": 1.2,
    "SCAN_READ_PAUSE": 0.15,
    "COMPLETE_TURN_TIME": 0.8,
    "AVOID_COOLDOWN_TIME": 2.0,
    "STUCK_THRESHOLD_COUNT": 3,
    "STUCK_WINDOW_TIME": 10.0,
    "STUCK_ESCAPE_TIME": 2.0,
    "MAX_SPEED_CM_S": 100,
    "WHEEL_BASE_CM": 30,
    "HEADING_TOLERANCE_RAD": 0.26,
    "EXPLORE_JITTER_TIME": 6.0,
    "EXPLORE_JITTER_AMOUNT": 0.10,
    "SONAR_FAIL_THRESHOLD": 5,
    "SONAR_FAIL_SPEED_SCALE": 0.3,
    "LOOP_HZ": 20,
}
BRAIN_CFG["OBSTACLE_THRESHOLD_CM"] = compute_obstacle_threshold(BRAIN_CFG)
