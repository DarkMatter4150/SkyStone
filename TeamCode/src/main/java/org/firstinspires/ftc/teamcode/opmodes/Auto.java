package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

/*
 * TODO
 * Foundation: turn it into the build zone, don't just pull it.
 */

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
//        RobotObject robotObject = new RobotObject(135, 111, 18, 18, 0);
//        encoderDrive = new EncoderDrive(robot, robotObject);

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
     * Experimental function to read instructions from a given filename
     */
    void readInstructionFile(String filename) {
        File file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/" + filename);
        Scanner scanner = null;
        try {
            scanner = new Scanner(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        ArrayList<String> data = new ArrayList<>();
        if (scanner != null) {
            while (scanner.hasNext()) {
                data.add(scanner.nextLine());
            }
        }

        for (String s : data) {
            float x;
            float y;
            int theta;
            int time;
            double speed;

            float xOffset;
            float yOffset;
            String target;

            String[] t0 = s.split(":");
            switch (t0[0]) {
                case "stones":
                    if (t0[1].equals("red")) {
                        skyStonesRed();
//                    } else {
//                        skyStonesBlue();
                    }
                    break;
                case "tabs":
                    if (t0[1].equals("0")) {
                        Log.i("ROBOT", "Close tabs");
                        robot.setTabs(false);
                        sleep(1000);
                    } else {
                        Log.i("ROBOT", "Open tabs");
                        robot.setTabs(true);
                        sleep(1000);
                    }
                    break;
                case "s": {
                    String[] t1 = t0[1].split(",");
                    x = Float.parseFloat(t1[0]);
                    y = Float.parseFloat(t1[1]);
                    theta = Integer.parseInt(t1[2]);

                    Log.i("ROBOT", String.format("Start: x=%f, y=%f, t=%d", x, y, theta));

                    RobotObject obj = new RobotObject(x, y, 18, 18, theta);
                    resetRobotObject(obj);
                    break;
                }
                case "c": {
                    String[] t1 = t0[1].split(",");
                    x = Float.parseFloat(t1[0]);
                    y = Float.parseFloat(t1[1]);
                    theta = Integer.parseInt(t1[2]);
                    theta -= robotObject.getRotation();
                    time = Integer.parseInt(t1[3]);
                    speed = Double.parseDouble(t1[4]);

                    Log.i("ROBOT", "Start Pos: " + robotObject.getCenterX() + " " + robotObject.getCenterY());

                    Log.i("ROBOT", String.format("Coord: x=%f, y=%f, t=%d, m=%d", x, y, theta, time));

                    moveCoord(x, y, theta, 100, time, speed);
                    break;
                }
                case "t": {
                    Log.i("ROBOT", t0[1]);
                    String[] t1 = t0[1].split(",");
                    target = t1[0];
                    xOffset = Float.parseFloat(t1[1]);
                    yOffset = Float.parseFloat(t1[2]);
                    theta = Integer.parseInt(t1[3]);
                    theta -= robotObject.getRotation();
                    time = Integer.parseInt(t1[4]);
                    speed = Double.parseDouble(t1[5]);

                    Log.i("ROBOT", "Start Pos: " + robotObject.getCenterX() + " " + robotObject.getCenterY());

                    Log.i("ROBOT", String.format("Start: target=%s x=%f, y=%f, t=%d, m=%d", target, xOffset, yOffset, theta, time));

                    moveTarget(field.getObject(target), xOffset, yOffset, theta, 100, time, speed);
                    break;
                }
            }
            Log.i("ROBOT", "End Pos: " + robotObject.getCenterX() + " " + robotObject.getCenterY());
        }
    }

    /**
     * rotating tests
     */
    void rotate() {
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY(), 180, 1000, 10000, null);
    }

    /**
     * Method used to park under the bridge
     */
    void parkRed() {
        moveCoord(robotObject.getCenterX(), 72, 0, 0, 30000, null);
    }

    /**
     * Method used to park under the blue bridge
     */
    void parkBlue() {
        moveCoord(10, 72, 0, 0, 30000, null);
    }

    //TODO: test SkyStone code

    /**
     * Method used to get the SkyStone
     */
    void skyStonesRed() {
        Log.i("SKYSTONES", "entering skystonered");
        moveCoord(106, 57);
        Log.i("SKYSTONES", robotObject.getCenterX() + "," + robotObject.getCenterY());
        for (int i = 0; i <= 2; i++) {
            moveCoord(106, 49 - (i * 8), 0, 100, 2000, 0.5);
            Log.i("SKYSTONES", robotObject.getCenterX() + "," + robotObject.getCenterY());
            if (i < 2) {
                int i2 = 0;
                int check = 0;
                while (i2 < 20) {
                    if (checkColorLeft()) {
                        check++;
                    }
                    i2++;
                }

                if (check > 15) {
                    skyStonePosition = i + 1;
                    break;
                }
            } else {
                skyStonePosition = 3;
            }
            double angle = robot.getAngle();
            moveCoord(robotObject.getCenterX(), robotObject.getCenterY(), (int)Math.round(angle), 100, 1000, 0.7d);
            robotObject.setRotation(robotObject.getRotation() - (int)Math.round(angle));
        }
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY() - 8);
        Log.i("SKYSTONES", robotObject.getCenterX() + "," + robotObject.getCenterY());
        grabBlock(1000, 1000);
//        moveCoord(106, robotObject.getCenterY());
        Log.i("HELP", robotObject.getCenterX() + "," + robotObject.getCenterY() + "," + robotObject.getRotation());
        moveCoord(106, 90, 0, 100, 7000, 0.7d);
        Log.i("HELP", robotObject.getCenterX() + "," + robotObject.getCenterY() + "," + robotObject.getRotation());
        dropBlock(200, 1000);
        parkRed();
//        skyStones2Red();
    }

    /**
     * Method used to get the second SkyStone
     */
    void skyStones2Red() {
        if (skyStonePosition != 3) {
            moveCoord(116, 28 - (skyStonePosition * 8));
            moveCoord(106, robotObject.getCenterY());
            grabBlock(1000, 1000);
            moveCoord(116, robotObject.getCenterY());
            moveCoord(116, 90);
            dropBlock(1000, 1000);
        }
    }

    /**
     * Method used to get the foundation, starting at x=135, y=111
     */
    void getFoundationRed() {
        moveCoord(133, robotObject.getCenterY(), 0, 100, 1000, null);
        moveCoord(133, 122.75f, 0, 100, 1000, null);
        moveTarget(field.getObject("r foundation"), 14f, 0, 0, 100, 2000, null);
        robot.setTabs(false);
        pause(1000);
        moveCoord(133, 122.75f, 0, 100, 2000, null);
        robot.setTabs(true);
        pause(3000);
        moveCoord(133, 95, 0, 100, 2300, null);
    }

    /**
     * Moves foundation for the blue side
     */
    void getFoundationBlue() {
        moveCoord(11, robotObject.getCenterY(), 0, 100, 1000, null);
        moveCoord(11, 122.75f, 0, 100, 1000, null);
        moveTarget(field.getObject("b foundation"), -14f, 0, 0, 100, 2000, null);
        robot.setTabs(false);
        pause(1000);
        moveCoord(9, 122.75f, 90, 100, 2500, null);
        robot.setTabs(true);
        pause(3000);
        moveCoord(9, 95, 0, 100, 2300, null);
    }

    /**
     * Checks if the thing the color sensor is looking at is closer than 6.5 cm and not yellow
     *
     * @return True: SkyStone; False: Normal block
     */
    private boolean checkColorRight() {
        RevColorSensorV3 colorSensor = robot.getColorSensorRight();
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        DistanceSensor distanceSensor = robot.getDistanceSensorRight();
        float distance = EncoderDrive.toFloat(distanceSensor.getDistance(DistanceUnit.CM));
        boolean found = red - blue <= 30 && distance < 6.5f;

//        Log.i("COLOR", String.format("red %f, blue %f, green %d, distance %f, found %b", red, blue, -1, distance, found));

        return found;
    }

    /**
     * Checks if the thing the color sensor is looking at is closer than 6.5 cm and not yellow
     *
     * @return True: SkyStone; False: Normal block
     */
    private boolean checkColorLeft() {
        RevColorSensorV3 colorSensor = robot.getColorSensorLeft();
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        DistanceSensor distanceSensor = robot.getDistanceSensorLeft();
        float distance = EncoderDrive.toFloat(distanceSensor.getDistance(DistanceUnit.CM));
        boolean found = red - blue <= 30 && distance < 6.5f;

//        Log.i("COLOR", String.format("red %f, blue %f, green %d, distance %f, found %b", red, blue, -1, distance, found));

        return found;
    }

    /**
     * testing code
     */
    private void move(float left, float right) {
        if (opModeIsActive()) {
            Matrix move = new Matrix(2, 2);
            move.setValues(new float[]{left, right, left, right});
            int[] targets = encoderDrive.drive(move, null);
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
        moveCoord(robotObject.getCenterX(), robotObject.getCenterY(), degrees, 100, timer, null);
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
            int[] targets = encoderDrive.drive(driveMatrix.fin, null);
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
     * @param moveTimer        Milliseconds it should take for this move. If it never reaches the end position, it will stop after this time elapses.
     */
    private void moveCoord(float x, float y, int rotationDegrees, long waitMilliseconds, long moveTimer, Double speed) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(x, y, rotationDegrees);
            int[] targets = encoderDrive.drive(driveMatrix.fin, speed);
            ElapsedTime timer = new ElapsedTime();
            double time = timer.milliseconds();
            while (opModeIsActive() && (timer.milliseconds() - time < moveTimer) &&
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
    private void moveTarget(FieldObject target, float xOffset, float yOffset, int rotationDegrees, long waitMilliseconds, long movetimer, Double speed) {
        if (opModeIsActive()) {
            EncoderDrive.Data driveMatrix = encoderDrive.getDriveMatrix(target, xOffset, yOffset, rotationDegrees);
            int[] targets = encoderDrive.drive(driveMatrix.fin, speed);
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
    private void grabBlock(long workTime, long waitTime) {
        encoderDrive.pickUpBlock();
        pause(workTime);
        encoderDrive.stop2();
        pause(waitTime);
        encoderDrive.revert("grab");
        pause(workTime);
        encoderDrive.stop();
    }

    /**
     * Method to drop a block from the intake slot.
     *
     * @param workTime Milliseconds for how long to drive backwards and outtake.
     */
    private void dropBlock(long workTime, long waitTime) {
        encoderDrive.dropBlock();
        pause(workTime);
        encoderDrive.stop();
        pause(waitTime);
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
