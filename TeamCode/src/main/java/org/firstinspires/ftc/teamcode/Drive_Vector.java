package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive_Vector {

    // angle is angle the robot should travel, and ranges from 0 (Robot's right) to 2π radians
    // (0-360 degrees). Right is 0 radians (0 degrees). Forward is an angle of π/2 radians
    // (90 degrees). Left is π radians (180 degrees). Backward is 3π/2 radians (270 degrees).
    double angle;

    // magnitude is the magnitude of the how quickly the robot should travel.  We will use
    // angle and magnitude to calculate the correct powers to apply to the 4 drive motors,
    // each of which will range from -1 to 1.
    // For a tutorial, see: https://www.youtube.com/watch?v=gnSW2QpkGXQ (Method 2).
    double magnitude;

    // Constructor to initialize to 0
    public Drive_Vector() {
        angle = 0;
        magnitude = 0;
    }

    // Constructor to initialize to specific values.
    public Drive_Vector(double angle, double magnitude) {
        this.angle = angle;
        this.magnitude = magnitude;
    }

    // Method to calculate angle and magnitude from a Gamepad
    public void getDriveVector (Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        angle = Math.atan2(y, x);
        magnitude = Math.hypot(x, y);
    }
}
