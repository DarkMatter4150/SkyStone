package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.breakout.EncoderDrive;
import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.field.FieldObject;
import org.firstinspires.ftc.teamcode.field.RobotObject;
import org.firstinspires.ftc.teamcode.matrix.Matrix;

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private Robot robot = new Robot(telemetry);   // Use a Pushbot's hardware
    private RobotObject robotObject;
    private ElapsedTime runtime = new ElapsedTime();
    private Field field = new Field();
    private EncoderDrive encoderDrive;
    private int skyStonePosition = -1;

    /**
     * Starts when you initialize the program. Pauses at waitForStart()
     */
    @Override
    public void runOpMode() {
        // Creates robotObject and encoderDrive to be used later. Set the starting position of the robot here.
        RobotObject robotObject = new RobotObject(135, 111, 18, 18, 0);
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

        instructions();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Run this and pass in the new {@link RobotObject} for which side you're on.
     * <p>
     * Without this, it defaults to red side.
     */
    void resetRobotObject(RobotObject obj) {
        robotObject = obj;
        encoderDrive = new EncoderDrive(robot, robotObject);
    }

    /**
     * Override this with instructions to do during autonomous
     * <p>
     * Methods used inside this are defined here and contain the specific move coordinates and objects
     * for each of the moves. We will override this with separate methods for red and blue autonomous.
     */
    void instructions() {
    }

    /**
     * rotating tests
     */
    void rotate() {
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY(), 180, 1000, 10000);
    }

    /**
     * Method used to park under the bridge
     */
    void parkRed() {
        moveCoord(134, 72, 0, 0, 30000);
    }

    /**
     * Method used to park under the blue bridge
     */
    void parkBlue() {
        moveCoord(10, 72, 0, 0, 30000);
    }

    //TODO: test SkyStone code

    /**
     * Method used to get the SkyStone
     */
    void skyStonesRed() {
        moveCoord(135, 35);
        moveCoord(107, 35);
        moveCoord(107, 25);
        for (int i = 0; i < 2; i++) {
            moveCoord(107, 25 - (i * 8));
            if (checkColor()) {
                skyStonePosition = i + 1;
                break;
            }
        }
        if (skyStonePosition == -1) {
            skyStonePosition = 3;
        }
        grabBlock(1000);
        moveCoord(116, robotObject.getCenterY());
        moveCoord(116, 90);
        dropBlock(1000);
    }

    /**
     * Method used to get the second SkyStone
     */
    void skyStones2Red() {
        moveCoord(135, 35);
        moveCoord(107, (skyStonePosition * 8) + 24);
        grabBlock(1000);
        moveCoord(116, robotObject.getCenterY());
        moveCoord(116, 90);
        dropBlock(1000);
    }

    public void mitchTest() {
        moveCoord(robotObject.getCenterX() + 24, robotObject.getCenterY(), 0, 1000, 10000);
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY() + 24, 0, 1000, 10000);
    }

    /**
     * Method used to get the foundation, starting at x=135, y=111
     */
    void getFoundationRed() {
        moveCoord(135, 122.75f, 0, 100, 1000);
        moveTarget(field.getObject("r foundation"), 14f, 0, 0, 100, 2000);
        robot.setTabs(false);
        pause(1000);
        moveCoord(135, 122.75f, 0, 100, 2000);
        robot.setTabs(true);
        pause(6000);
//        moveCoord(135, 95, 0, 100, 2300);
//        moveCoord(95, 95, 0, 100, 2300);
//        robot.setTabs(false);
//        rotate(180, 2500);
//        moveCoord(95, 125, 0, 100, 2000);
//        moveCoord(115, 125, 0, 100, 2000);
    }

    /**
     * Moves foundation for the blue side
     */
    void getFoundationBlue() {
        moveCoord(9, 122.75f, 0, 100, 1000);
        moveTarget(field.getObject("b foundation"), -14f, 0, 0, 100, 2000);
        robot.setTabs(false);
        pause(1000);
        moveCoord(9, 122.75f, 0, 100, 2000);
        robot.setTabs(true);
        pause(6000);
//        moveCoord(9, 95, 0, 100, 2300);
//        moveCoord(49, 95, 0, 100, 2300);
//        robot.setTabs(false);
//        rotate(180, 2500);
//        moveCoord(49, 125, 0, 100, 2000);
//        moveCoord(29, 125, 0, 100, 2000);
    }

    /**
     * Checks if the thing the color sensor is looking at is closer than 6.5 cm and not yellow
     *
     * @return True: SkyStone; False: Normal block
     */
    private boolean checkColor() {
        ColorSensor colorSensor = robot.getColorSensor();
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        DistanceSensor distanceSensor = robot.getDistanceSensor();
        float distance = EncoderDrive.toFloat(distanceSensor.getDistance(DistanceUnit.CM));

        return !((red - blue > 30) && (distance < 6.5f) && (red > blue));
    }

    /**
     * testing code
     */
    private void move(float left, float right) {
        if (opModeIsActive()) {
            Matrix move = new Matrix(2, 2);
            move.setValues(new float[]{left, right, left, right});
            int[] targets = encoderDrive.drive(move);
            while (opModeIsActive() &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) || robot.isBusy(Robot.Motor.FRONT_RIGHT) ||
                            robot.isBusy(Robot.Motor.BACK_LEFT) || robot.isBusy(Robot.Motor.BACK_RIGHT))) {
                encoderDrive.tick(telemetry, targets, new float[]{});
            }
        }
    }

    /**
     * Rotates the robot by the degrees given for the milliseconds given
     *
     * @param degrees int for number of degrees to rotate
     * @param timer   int for number of milliseconds to do the move for
     */
    private void rotate(int degrees, int timer) {
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY(), degrees, 100, timer);
    }

    /**
     * Used to move to a specific coordinate on the field.
     *
     * @param x X coordinate in inches for where to move the center of the robot to.
     * @param y Y coordinate in inches for where to move the center of the robot to.
     */
    private void moveCoord(float x, float y) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(x, y, 0);
            int[] targets = encoderDrive.drive(driveMatrix.fin);
            ElapsedTime timer = new ElapsedTime();
            double time = timer.milliseconds();
            while (opModeIsActive() && (timer.milliseconds() - time < 1000) &&
                    (robot.isBusy(Robot.Motor.FRONT_LEFT) || robot.isBusy(Robot.Motor.FRONT_RIGHT) ||
                            robot.isBusy(Robot.Motor.BACK_LEFT) || robot.isBusy(Robot.Motor.BACK_RIGHT))) {
                encoderDrive.tick(telemetry, targets, driveMatrix.telemetryData);
            }
            encoderDrive.stop();
            sleep(0);
        }
    }

    /**
     * Used to move to a specific coordinate on the field.
     *
     * @param x                X coordinate in inches for where to move the center of the robot to.
     * @param y                Y coordinate in inches for where to move the center of the robot to.
     * @param rotationDegrees  Integer of degrees to rotate the robot for.
     * @param waitMilliseconds Milliseconds to wait after movement before starting next movement.
     * @param movetimer        Milliseconds it should take for this move. If it never reaches the end position, it will stop after this time elapses.
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
                encoderDrive.tick(telemetry, targets, driveMatrix.telemetryData);
            }
            encoderDrive.stop();
            pause(waitMilliseconds);
        }
    }

    /**
     * Used to move to a specific object on the field.
     *
     * @param target           Target {@link FieldObject} to move the center of the robot to.
     * @param xOffset          Inches to move the destination point after rotation on the x axis.
     * @param yOffset          Inches to move the destination point after rotation on the y axis.
     * @param rotationDegrees  Integer of degrees to rotate the robot for.
     * @param waitMilliseconds Milliseconds to wait after movement before starting next movement.
     * @param movetimer        Milliseconds it should take for this move. If it never reaches the end position, it will stop after this time elapses.
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
                encoderDrive.tick(telemetry, targets, driveMatrix.telemetryData);
            }
            encoderDrive.stop();
            pause(waitMilliseconds);
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
        double time = runtime.milliseconds();
        while (opModeIsActive()) {
            if (runtime.milliseconds() - time > workTime) {
                break;
            }
        }
    }
}
