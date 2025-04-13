package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

// IF you see this, it updated on Sun April 13 about 13:18, Central

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

    private static class DrivePowers {

        // Declare constants
        static double ROTATION_SCALE = 0.75;   // Scaling factor to determine "twisting" effect on power levels
        static double MAX_POWER_INCREASE = 0.05;   // Maximum power increase (acceleration) per update
        static double MAX_POWER_DECREASE = -0.05;   // Maximum power decrease (deceleration) per update

        double currentPowerLF;  // Current power for the robot's left front drive motor
        double currentPowerLR;  // Current power for the robot's left rear drive motor
        double currentPowerRF;  // Current power for the robot's right front drive motor
        double currentPowerRR;  // Current power for the robot's right rear drive motor

        // Class constructor
        private DrivePower() {

            currentPowerLF = 0;  // Initialize current power for the robot's left front drive motor
            currentPowerLR = 0;  // Initialize current power for the robot's left rear drive motor
            currentPowerRF = 0;  // Initialize current power for the robot's right front drive motor
            currentPowerRR = 0;  // Initialize current power for the robot's right rear drive motor

        }
        // Method to read the sticks on controller 1 and update the drive motors' power
        private void updateDrivePower(Gamepad gamepad1) {

            // The driver controls the robot using:
            //  1.  Linear motion using the left stick on controller 1 with "drive" (forward/backward) and
            //      "strafe" (left/right), and
            //  2.  Rotational motion using the right stick on controller 1 with "twist" (rotation).
            //
            // Read the left and right sticks of Gamepad 1.
            double drive = UtilsLib.deadStick(gamepad1.left_stick_y);
            double strafe = UtilsLib.deadStick(gamepad1.left_stick_x);
            double twist = UtilsLib.deadStick(gamepad1.right_stick_x);

            // Calculate the difference between the current power levels and the target power levels calculated
            // from sticks on gamepad1. These can be positive (acceleration) or negative (deceleration).
            double deltaPowerLF = currentPowerLF - (drive + strafe + (ROTATION_SCALE * twist));
            double deltaPowerRF = currentPowerRF - (drive - strafe - (ROTATION_SCALE * twist));
            double deltaPowerLR = currentPowerLR - (drive - strafe + (ROTATION_SCALE * twist));
            double deltaPowerRR = currentPowerRR - (drive + strafe - (ROTATION_SCALE * twist));

            // If delta > 0 (acceleration), add the lesser of the delta and the maximum power increase.
            // If delta <= 0 (deceleration), add the greater of the delta and the maximum power decrease
            // (we "add the greater of" because both delta <=0 and MAX_POWER_DECREASE < 0).
            if (deltaPowerLF > 0) {
                currentPowerLF += Math.min(MAX_POWER_INCREASE, deltaPowerLF);
            } else {
                currentPowerLF += Math.max(MAX_POWER_DECREASE, deltaPowerLF);
            }
            if (deltaPowerRF > 0) {
                currentPowerRF += Math.min(MAX_POWER_INCREASE, deltaPowerRF);
            } else {
                currentPowerRF += Math.max(MAX_POWER_DECREASE, deltaPowerRF);
            }
            if (deltaPowerLR > 0) {
                currentPowerLR += Math.min(MAX_POWER_INCREASE, deltaPowerLR);
            } else {
                currentPowerLR += Math.max(MAX_POWER_DECREASE, deltaPowerLR);
            }
            if (deltaPowerRR > 0) {
                currentPowerRR += Math.min(MAX_POWER_INCREASE, deltaPowerRR);
            } else {
                currentPowerRR += Math.max(MAX_POWER_DECREASE, deltaPowerRR);
            }

            // Determine the maximum target power level among the drive motors so we can normalize
            // the power levels if one or more of them is greater than 1.
            double max = Math.max(Math.abs(currentPowerLF), Math.abs(currentPowerRF));
            max = Math.max(max, Math.abs(currentPowerLR));
            max = Math.max(max, Math.abs(currentPowerRR));

            // If the maximum power level for any motor is greater than 1, then normalize the power
            // levels so the maximum power level is 1.
            if (max > 1.0) {
                currentPowerLF  /= max;
                currentPowerRF /= max;
                currentPowerLR   /= max;
                currentPowerRR  /= max;
            }




        }

    }

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

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // These are the current power setting for each of the 4 drive motors.
        double currentLeftFrontPower = 0;
        double currentRightFrontPower = 0;
        double currentLeftBackPower = 0;
        double currentRightBackPower = 0;

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            // Set the motors to the calculated values.
            front_left.setPower(currentLeftFrontPower);
            front_right.setPower(currentRightFrontPower);
            back_left.setPower(currentLeftBackPower);
            back_right.setPower(currentRightBackPower);

        }


    }
}