package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// IF you see this, it updated

/*
This code uses the LinearOpMode rather than OpMode.

See https://gm0.org/en/latest/docs/software/getting-started/linear-opmode-vs-opmode.html
for a description of the differences between LinearOpMode and Opmode.

    runOpMode():        Code inside this method will run exactly once after you press the
                        INIT button. This is where you should put all code for the OpMode.

    waitForStart():     This method pauses the Op-Mode until you press the START button on
                        the driver station.

    isStarted():        Returns true if the START button has been pressed, otherwise it
                        returns false.

    isStopRequested():  Returns true if the STOP button has been pressed, otherwise it
                        returns false.

    idle():             Calls Thread.yield, allowing other threads at the same priority
                        level to run.

    opModeIsActive():   Returns isStarted() && !isStopRequested() and calls idle().

    opModeInInit():     Returns !isStarted() && !isStopRequested() and does not call idle().

*/
@TeleOp(name="Teleop Dev01", group="19380")
public class Robot_2025 extends LinearOpMode {

    // Declare and initialize DcMotors.
    private DcMotor front_left = null;  // Front left drive motor
    private DcMotor front_right = null;  // Front right drive motor
    private DcMotor back_left = null;  // Back left drive motor
    private DcMotor back_right = null;  // Back right drive motor

    // runOpMode() runs exactly once when the INIT button is pressed.
    // All code for the OpMode goes in here.
    @Override
    public void runOpMode() {

        // Name strings must match the config names on the Robot Controller app.
        front_left = hardwareMap.get(DcMotor.class, "fldrive");
        front_right = hardwareMap.get(DcMotor.class, "frdrive");
        back_left = hardwareMap.get(DcMotor.class, "bldrive");
        back_right = hardwareMap.get(DcMotor.class, "brdrive");

        // Left and right motor must turn in opposite directions because they
        // have mirror symmetry.
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // These are the current power setting for each of the 4 drive motors.
        double currentLeftFrontPower = 0;
        double currentRightFrontPower = 0;
        double currentLeftBackPower = 0;
        double currentRightBackPower = 0;

        // This is the scaling factor to determines "twisting" effect one the power levels
        double rotationScale = 0.75;

        // These are the maximum power increase and decrease per loop to provide a more gradual
        // acceleration and deceleration rather than simply setting the powers to 1.
        double MAX_POWER_INCREASE = 0.05;
        double MAX_POWER_DECREASE = -0.05;

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            // The driver controls the robot using:
            //  1.  Linear motion with drive (forward/backward) and strafe (left/right)
            //      using the left stick on controller 1, and
            //  2.  Rotational motion (rotating the whole chassis) using the
            //      right stick on controller 1.

            // Read the left and right sticks of Gampad 1.
            double drive = UtilsLib.deadStick(gamepad1.left_stick_y);
            double strafe = UtilsLib.deadStick(gamepad1.left_stick_x);
            double twist = UtilsLib.deadStick(gamepad1.right_stick_x);

            // Calculate the difference between the current power levels and the target power levels calculated
            // from sticks on gamepad1. These can be positive (acceleration) or negative (deceleration).
            double deltaLeftFrontPower = currentLeftFrontPower - (drive + strafe + (rotationScale * twist));
            double deltaRightFrontPower = currentRightFrontPower - (drive - strafe - (rotationScale * twist));
            double deltaLeftBackPower = currentLeftBackPower - (drive - strafe + (rotationScale * twist));
            double deltaRightBackPower = currentRightBackPower - (drive + strafe - (rotationScale * twist));

            // If delta > 0 (acceleration), add the lesser of the delta and the maximum power increase.
            // If delta <= 0 (deceleration), add the greater of the delta and the maximum power decrease
            // (we "add the greater of" because both delta <=0 and MAX_POWER_DECREASE < 0).
            if (deltaLeftFrontPower > 0) {
                currentLeftFrontPower += Math.min(MAX_POWER_INCREASE, deltaLeftFrontPower);
            } else {
                currentLeftFrontPower += Math.max(MAX_POWER_DECREASE, deltaLeftFrontPower);
            }
            if (deltaRightFrontPower > 0) {
                currentRightFrontPower += Math.min(MAX_POWER_INCREASE, deltaRightFrontPower);
            } else {
                currentRightFrontPower += Math.max(MAX_POWER_DECREASE, deltaRightFrontPower);
            }
            if (deltaLeftBackPower > 0) {
                currentLeftBackPower += Math.min(MAX_POWER_INCREASE, deltaLeftBackPower);
            } else {
                currentLeftBackPower += Math.max(MAX_POWER_DECREASE, deltaLeftBackPower);
            }
            if (deltaRightBackPower > 0) {
                currentRightBackPower += Math.min(MAX_POWER_INCREASE, deltaRightBackPower);
            } else {
                currentRightBackPower += Math.max(MAX_POWER_DECREASE, deltaRightBackPower);
            }

            // Determine the maximum target power level among the drive motors so we can normalize
            // the power levels if one or more of them is greater than 1.
            double max = Math.max(Math.abs(currentLeftFrontPower), Math.abs(currentRightFrontPower));
            max = Math.max(max, Math.abs(currentLeftBackPower));
            max = Math.max(max, Math.abs(currentRightBackPower));

            // If the maximum power level for any motor is greater than 1, then normalize the power
            // levels so the maximum power level is 1.
            if (max > 1.0) {
                currentLeftFrontPower  /= max;
                currentRightFrontPower /= max;
                currentLeftBackPower   /= max;
                currentRightBackPower  /= max;
            }

            // Set the motors to the calculated values.
            front_left.setPower(currentLeftFrontPower);
            front_right.setPower(currentRightFrontPower);
            back_left.setPower(currentLeftBackPower);
            back_right.setPower(currentRightBackPower);

        }


    }
}