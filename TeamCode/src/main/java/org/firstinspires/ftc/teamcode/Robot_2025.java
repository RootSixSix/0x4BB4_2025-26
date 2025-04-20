package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.TRUE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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

    private static final Logger log = LoggerFactory.getLogger(Robot_2025.class);
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightRearMotor = null;

    // These are the current power setting for each of the 4 drive motors.
    double leftFrontPower = 0;  // Current power for the robot's left front drive motor
    double rightFrontPower = 0;  // Current power for the robot's right front drive motor
    double leftRearPower = 0;  // Current power for the robot's left rear drive motor
    double rightRearPower = 0;  // Current power for the robot's right rear drive motor

    // Initialize log file
    Log logFile = new Log("logFile.txt",TRUE);

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
        logFile.addData(drive);
        logFile.addData(strafe);
        logFile.addData(twist);

        // Calculate the differences between:
        //  1.  the target power levels calculated in the parentheticals using the values read from
        //      the sticks on gamepad1, and
        //  2.  the current power levels.
        // These can be positive (acceleration) or negative (deceleration).  For a description of
        // the mecanum drive calculations, see:
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        double deltaLeftFront = (drive + strafe + (ROTATION_SCALE * twist)) - leftFrontPower;
        double deltaRightFront = (drive - strafe - (ROTATION_SCALE * twist)) - rightFrontPower;
        double deltaLeftRear = (drive - strafe + (ROTATION_SCALE * twist)) - leftRearPower;
        double deltaRightRear = (drive + strafe - (ROTATION_SCALE * twist)) - rightRearPower;
        logFile.addData(drive + strafe + (ROTATION_SCALE * twist));
        logFile.addData(drive - strafe - (ROTATION_SCALE * twist));
        logFile.addData(drive - strafe + (ROTATION_SCALE * twist));
        logFile.addData(drive + strafe - (ROTATION_SCALE * twist));
        logFile.addData(deltaLeftFront);
        logFile.addData(deltaRightFront);
        logFile.addData(deltaLeftRear);
        logFile.addData(deltaRightRear);

        // If delta > 0 (acceleration), add the lesser of the delta and the maximum power increase.
        // If delta <= 0 (deceleration), add the greater of the delta and the maximum power decrease
        // (we "add the greater of" because both delta <=0 and MAX_POWER_DECREASE < 0).
        if (deltaLeftFront > 0) {
            leftFrontPower += Math.min(MAX_POWER_INCREASE, deltaLeftFront);
        } else {
            leftFrontPower += Math.max(MAX_POWER_DECREASE, deltaLeftFront);
        }
        if (deltaRightFront > 0) {
            rightFrontPower += Math.min(MAX_POWER_INCREASE, deltaRightFront);
        } else {
            rightFrontPower += Math.max(MAX_POWER_DECREASE, deltaRightFront);
        }
        if (deltaLeftRear > 0) {
            leftRearPower += Math.min(MAX_POWER_INCREASE, deltaLeftRear);
        } else {
            leftRearPower += Math.max(MAX_POWER_DECREASE, deltaLeftRear);
        }
        if (deltaRightRear > 0) {
            rightRearPower += Math.min(MAX_POWER_INCREASE, deltaRightRear);
        } else {
            rightRearPower += Math.max(MAX_POWER_DECREASE, deltaRightRear);
        }
        logFile.addData(leftFrontPower);
        logFile.addData(rightFrontPower);
        logFile.addData(leftRearPower);
        logFile.addData(rightRearPower);

        // Determine the maximum target power level among the drive motors so we can normalize
        // the power levels if one or more of them is greater than 1.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        // If the maximum power level for any motor is greater than 1, then normalize the power
        // levels so the maximum power level is 1.
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        // Set the drive motor powers
        leftFrontMotor.setPower(DRIVE_SCALE * leftFrontPower);
        rightFrontMotor.setPower(DRIVE_SCALE * rightFrontPower);
        leftRearMotor.setPower(DRIVE_SCALE * leftRearPower);
        rightRearMotor.setPower(DRIVE_SCALE * rightRearPower);

        logFile.update();

    }

    // runOpMode() runs exactly once when the INIT button is pressed.
    // All code for the OpMode goes in here.
    @Override
    public void runOpMode() {

        /// Initialize everything that should be initialized before the player presses START
        /// on the Driver's Hub.

        // Name strings (e.g., "fldrive") must match the configuration names on the Robot Controller
        // app on the Driver Hub.
        leftFrontMotor = hardwareMap.get(DcMotor.class, "fldrive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "frdrive");
        leftRearMotor = hardwareMap.get(DcMotor.class, "bldrive");
        rightRearMotor = hardwareMap.get(DcMotor.class, "brdrive");

        // Left and right motors must turn in opposite directions because the wheels have mirror
        // symmetry.
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        /// Wait for the game driver to press play
        waitForStart();

        /// Initialize variables that should be initialized after the player hits START on the
        /// Driver Hub but before we enter the MAIN GAME LOOP.

       // Initialize the time we start the MAIN GAME LOOP.
        long mainLoopStartTime = System.nanoTime();

        // We use current and previous Gamepad objects so that we can detect changes in the
        // state of the gamepads during the "while (opModeIsActive()) {}" loop. For example, if
        // the current state of gamepad1 is that button A is pressed and the previous state is
        // that button A is not pressed, then we know that the driver pressed button A after
        // the beginning of the last loop but before the beginning of the current loop.
        // See https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ///
        ///  MAIN GAME LOOP: Runs until the driver presses stop
        ///
        while (opModeIsActive()) {

            // Store the currentGamepad1/2 values (which were used last loop iteration) in
            // previousGamepad1/2 so we can detect state changes for this iteration.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values to use for this loop iteration in currentGamepad1/2.
            // This prevents the gamepad values from changing between being used and stored
            // in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Update the drive powers based on gamepad 1
            updateDrivePowers(currentGamepad1);

        }


    }
}