package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.EncoderDrive;
import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldObject;
import org.firstinspires.ftc.teamcode.field.RobotObject;

/**
 * Autonomous for starting on the red side.
 */
@Autonomous(name = "Red Autonomous", group = "Pushbot")
public class RedAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private Robot robot = new Robot(telemetry);   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private Field field = new Field();
    private EncoderDrive encoderDrive;

    /**
     * Starts when you initialize the program. Pauses at waitForStart()
     */
    @Override
    public void runOpMode() {
        // Creates robotObject and encoderDrive to be used later. Set the starting position of the robot here.
        RobotObject robotObject = new RobotObject(126, 102, 18, 18, 0);
        encoderDrive = new EncoderDrive(robot, robotObject);

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Sets the run mode of each motor and resets the encoders.
        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d : %7d : %7d : %7d",
                robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        moveCoord(135, 120, 0, 1000, 1000);
        moveCoord(126, 120, 0, 1000, 1000);
        moveCoord(126, 111, 0, 1000, 1000);
        moveCoord(135, 111, 0, 1000, 1000);

//        moveTarget(field.getObject("r sky stones"), 0, 0, 0, 1000);
//        grabBlock(1000);
//        moveTarget(field.getObject("mid line"), -20, 12, 0, 0);
//        moveTarget(field.getObject("mid line"), 20, 12, 0, 1000);
//        dropBlock(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Used to move to a specific coordinate on the field.
     *
     * @param x X coordinate in inches for where to move the center of the robot to.
     * @param y Y coordinate in inches for where to move the center of the robot to.
     * @param rotationDegrees Integer of degrees to rotate the robot for.
     * @param waitMilliseconds Milliseconds to wait after movement before starting next movement.
     * @param movetimer Milliseconds it should take for this move. If it never reaches the end position, it will stop after this time elapses.
     */
    private void moveCoord(float x, float y, int rotationDegrees, long waitMilliseconds, long movetimer) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(x, y, rotationDegrees);
            int[] targets = encoderDrive.drive(driveMatrix.fin);
            ElapsedTime timer = new ElapsedTime();
            double time = timer.milliseconds();
            while (opModeIsActive() && (timer.milliseconds() - time < movetimer) &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) || robot.isBusy(Robot.Motor.FRONT_RIGHT) ||
                            robot.isBusy(Robot.Motor.BACK_LEFT) || robot.isBusy(Robot.Motor.BACK_RIGHT))) {
                encoderDrive.tick(telemetry, targets, driveMatrix.teledata);
            }
            encoderDrive.stop();
            sleep(waitMilliseconds);
        }
    }

    /**
     * Used to move to a specific object on the field.
     *
     * @param target Target {@link FieldObject} to move the center of the robot to.
     * @param xOffset Inches to move the destination point after rotation on the x axis.
     * @param yOffset Inches to move the destination point after rotation on the y axis.
     * @param rotationDegrees Integer of degrees to rotate the robot for.
     * @param waitMilliseconds Milliseconds to wait after movement before starting next movement.
     * @param movetimer Milliseconds it should take for this move. If it never reaches the end position, it will stop after this time elapses.
     */
    private void moveTarget(FieldObject target, float xOffset, float yOffset, int rotationDegrees, long waitMilliseconds, long movetimer) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(target, xOffset, yOffset, rotationDegrees);
            int[] targets = encoderDrive.drive(driveMatrix.fin);
            ElapsedTime timer = new ElapsedTime();
            double time = timer.milliseconds();
            while (opModeIsActive() && (timer.milliseconds() - time < movetimer) &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) || robot.isBusy(Robot.Motor.FRONT_RIGHT) ||
                            robot.isBusy(Robot.Motor.BACK_LEFT) || robot.isBusy(Robot.Motor.BACK_RIGHT))) {
                encoderDrive.tick(telemetry, targets, driveMatrix.teledata);
            }
            encoderDrive.stop();
            sleep(waitMilliseconds);
        }
    }

    /**
     * Method to grab a block into the intake slot.
     *
     * @param workTime Milliseconds for how long to drive forward and intake.
     */
    private void grabBlock(long workTime) {
        encoderDrive.pickUpBlock();
        pause(workTime);
        encoderDrive.revert("grab");
        pause(workTime);
        encoderDrive.stop();
    }

    /**
     * Method to drop a block from the intake slot.
     *
     * @param workTime Milliseconds for how long to drive backwards and outtake.
     */
    private void dropBlock(long workTime) {
        encoderDrive.dropBlock();
        pause(workTime);
        encoderDrive.revert("drop");
        pause(workTime);
        encoderDrive.stop();
    }

    /**
     * Pauses to allow something to happen.
     *
     * @param workTime Milliseconds to wait before continuing.
     */
    private void pause(long workTime) {
        time = runtime.milliseconds();
        while (opModeIsActive()) {
            if (runtime.milliseconds() - time > workTime) {
                break;
            }
        }
    }
}
