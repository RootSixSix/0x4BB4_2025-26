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

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    // These are the current power setting for each of the 4 drive motors.
    double leftFront = 0;  // Current power for the robot's left front drive motor
    double leftRear = 0;  // Current power for the robot's left rear drive motor
    double rightFront = 0;  // Current power for the robot's right front drive motor
    double rightRear = 0;  // Current power for the robot's right rear drive motor

    // Method to read the sticks on gamepad 1 and update the drive motors' power
    public void updateDrivePowers(Gamepad gamepad1) {

        // Declare constants
        double DRIVE_SCALE = 1.00;          // Scaling factor to scale normalized drive powers
        double ROTATION_SCALE = 0.75;       // Scaling factor to determine "twisting" effect on power levels
        double MAX_POWER_INCREASE = 0.05;   // Maximum power increase (acceleration) per update
        double MAX_POWER_DECREASE = -0.05;  // Maximum power decrease (deceleration) per update

        // The driver controls the robot using:
        //  1.  Linear motion using the left stick on controller 1 with "drive" (forward/backward) and
        //      "strafe" (left/right), and
        //  2.  Rotational motion using the right stick on controller 1 with "twist" (rotation).
        //
        // Read the left and right sticks of Gamepad 1.  The deadStick method returns 0 if
        // -0.05 >= value of stick <= 0.05.
        double drive = UtilsLib.deadStick(gamepad1.left_stick_y);
        double strafe = UtilsLib.deadStick(gamepad1.left_stick_x);
        double twist = UtilsLib.deadStick(gamepad1.right_stick_x);

        // Calculate the differences between:
        //  1.  the target power levels calculated in the parentheticals using the values read from
        //      the sticks on gamepad1, and
        //  2.  the current power levels.
        // These can be positive (acceleration) or negative (deceleration).  For a description of
        // the mecanum drive calculations, see:
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        double deltaLeftFront = (drive + strafe + (ROTATION_SCALE * twist)) - leftFront;
        double deltaRightFront = (drive - strafe - (ROTATION_SCALE * twist)) - rightFront;
        double deltaLeftRear = (drive - strafe + (ROTATION_SCALE * twist)) - leftRear;
        double deltaRightRear = (drive + strafe - (ROTATION_SCALE * twist)) - rightRear;

        // If delta > 0 (acceleration), add the lesser of the delta and the maximum power increase.
        // If delta <= 0 (deceleration), add the greater of the delta and the maximum power decrease
        // (we "add the greater of" because both delta <=0 and MAX_POWER_DECREASE < 0).
        if (deltaLeftFront > 0) {
            leftFront += Math.min(MAX_POWER_INCREASE, deltaLeftFront);
        } else {
            leftFront += Math.max(MAX_POWER_DECREASE, deltaLeftFront);
        }
        if (deltaRightFront > 0) {
            rightFront += Math.min(MAX_POWER_INCREASE, deltaRightFront);
        } else {
            rightFront += Math.max(MAX_POWER_DECREASE, deltaRightFront);
        }
        if (deltaLeftRear > 0) {
            leftRear += Math.min(MAX_POWER_INCREASE, deltaLeftRear);
        } else {
            leftRear += Math.max(MAX_POWER_DECREASE, deltaLeftRear);
        }
        if (deltaRightRear > 0) {
            rightRear += Math.min(MAX_POWER_INCREASE, deltaRightRear);
        } else {
            rightRear += Math.max(MAX_POWER_DECREASE, deltaRightRear);
        }

        // Determine the maximum target power level among the drive motors so we can normalize
        // the power levels if one or more of them is greater than 1.
        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftRear));
        max = Math.max(max, Math.abs(rightRear));

        // If the maximum power level for any motor is greater than 1, then normalize the power
        // levels so the maximum power level is 1.
        if (max > 1.0) {
            leftFront  /= max;
            rightFront /= max;
            leftRear   /= max;
            rightRear  /= max;
        }

        front_left.setPower(DRIVE_SCALE * leftFront);
        front_right.setPower(DRIVE_SCALE * rightFront);
        back_left.setPower(DRIVE_SCALE * leftRear);
        back_right.setPower(DRIVE_SCALE * rightRear);

    }

    // runOpMode() runs exactly once when the INIT button is pressed.
    // All code for the OpMode goes in here.
    @Override
    public void runOpMode() {

        // Name strings must match the config names on the Robot Controller app.
        // Declare and initialize DcMotors.
        front_left = hardwareMap.get(DcMotor.class, "fldrive");
        front_right = hardwareMap.get(DcMotor.class, "frdrive");
        back_left = hardwareMap.get(DcMotor.class, "bldrive");// Back right drive motor
        back_right = hardwareMap.get(DcMotor.class, "brdrive");

        // Left and right motor must turn in opposite directions because they
        // have mirror symmetry.
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // Declare and initialize current and previous gamepad objects.  We use current
        // and previou objects so that we can detect changes in the state of the gamepads
        // during the "while (opModeIsActive()) {}" loop. For example, if the current state
        // of gamepad1 is that the A button is pressed and the previous state is that the
        // A button is not pressed, then we know that the driver pressed the A button during
        // last loop.  See https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            // Store the currentGamepad1/2 values (which were used last loop iteration) in
            // previousGamepad1/2 so we can detect state changes for this iteration.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in currentGamepad1/2 to be
            // used for this loop iteration. This prevents the gamepad values from changing
            // between being used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Update the drive powers based on gamepad 1
            updateDrivePowers(currentGamepad1);

        }


    }
}