/**
 * @file global.h
 * @brief Global singleton instances (extern declarations).
 */
#pragma once

/* Drivers */
class Motor; extern Motor mt;
class Buzzer; extern Buzzer bz;
class LED; extern LED led;
class Button; extern Button btn;
class IMU; extern IMU imu;
class Encoder; extern Encoder enc;
class Ultrasonic; extern Ultrasonic us;

/* Core modules */
class SpeedController; extern SpeedController sc;
class WallDetector; extern WallDetector wd;
class SearchRun; extern SearchRun sr;
class FastRun; extern FastRun fr;
class MazeSolver; extern MazeSolver ms;
class Emergency; extern Emergency em;
class UserInterface; extern UserInterface ui;