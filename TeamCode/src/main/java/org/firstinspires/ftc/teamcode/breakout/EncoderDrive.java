package org.firstinspires.ftc.teamcode.breakout;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.field.FieldObject;
import org.firstinspires.ftc.teamcode.field.RobotObject;
import org.firstinspires.ftc.teamcode.matrix.Matrix;
import org.firstinspires.ftc.teamcode.matrix.MatrixHandler;

public class EncoderDrive {

    private Robot robot;
    private RobotObject robotObject;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 1120;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;      // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final float DRIVE_SPEED = 0.7f;

    private static final double TAU = 2 * Math.PI;
    private final static float RADIUS = 25.456f;

    /**
     * Constructor. Needs the {@link Robot} and {@link RobotObject} objects from the autonomous
     * program so that it can drive.
     *
     * @param robot {@link Robot} object used for getting the motors.
     * @param robotObject {@link RobotObject} object used for getting the robot's position.
     */
    public EncoderDrive(Robot robot, RobotObject robotObject) {
        this.robot = robot;
        this.robotObject = robotObject;
    }

    /**
     * Creates a drive matrix for the wheels to know how far to travel. Used for the drive method.
     * Uses X and Y Coordinates for the destination.
     *
     * @param destX           X Coordinate that you want to go to. Calculated against center of robot.
     * @param destY           Y Coordinate that you want to go to. Calculated against center of robot.
     * @param rotationDegrees How many degrees you want to rotate.
     * @return {@link Data} object containing telemetry data and the drive matrix for the drive method.
     */
    public Data getDriveMatrix(float destX, float destY, float rotationDegrees) {
        return matrix(destX, destY, 0, 0, rotationDegrees);
    }

    /**
     * Creates a drive matrix for the wheels to know how far to travel. Used for the drive method.
     * Uses a {@link FieldObject} to get the X and Y.
     *
     * @param destination     {@link FieldObject} used to find the coordinates of where you want the robot to go. Calculated against center of robot and object, use offsets to change that.
     * @param xOffset         Offset used to move the destination X coordinate calculated before rotation. (If 0 the destination X will be the center of the object)
     * @param yOffset         Offset used to move the destination Y coordinate calculated before rotation. (If 0 the destination Y will be the center of the object)
     * @param rotationDegrees How many degrees you want to rotate.
     * @return {@link Data} object containing telemetry data and the drive matrix for the drive method.
     */
    public Data getDriveMatrix(FieldObject destination, float xOffset, float yOffset, float rotationDegrees) {
        // destX and destY are the centers of the destination field object.
        float destX = destination.getX() + (destination.getWidth() / 2); //86.5
        float destY = destination.getY() + (destination.getLength() / 2); //122.75

        //104.75
        return matrix(destX, destY, xOffset, yOffset, rotationDegrees);
    }

    /**
     * Creates a drive matrix for the wheels to know how far to travel. Used for the drive method.
     *
     * @param x               X coordinate of destination to calculate the distance needed to travel. Calculated against center of robot.
     * @param y               Y coordinate of destination to calculate the distance needed to travel. Calculated against center of robot.
     * @param xOffset         Offset used to move the destination X coordinate calculated before rotation. (If 0 the destination X will be the x coordinate)
     * @param yOffset         Offset used to move the destination Y coordinate calculated before rotation. (If 0 the destination Y will be the y coordinate
     * @param rotationDegrees How many degrees you want to rotate.
     * @return Returns {@link Data} object with telemetry and the drive matrix.
     */
    private Data matrix(float x, float y, float xOffset, float yOffset, float rotationDegrees) {
        Log.i("ROBOT", "Start Coord (in drive): " + String.valueOf(robotObject.getCenterX()));
        Log.i("ROBOT", String.format("x=%f, y=%f, xoff=%f, yoff=%f, rotate=%f", x, y, xOffset, yOffset, rotationDegrees));
        // Sets robot to run with encoders.
        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

        //Offset application.
        x += xOffset;
        y += yOffset;

        // Sets up variables for the robot's center position.
        float robotL = robotObject.getLength();
        float robotW = robotObject.getWidth();
        float robotX = robotObject.getX();
        float robotY = robotObject.getY();
        float[] robotTopRight = {robotX + robotW, robotY + robotL};
        float[] robotBottomLeft = {robotX, robotY};
        float robotCenterX = (robotBottomLeft[0] + robotTopRight[0]) / 2;
        float robotCenterY = (robotBottomLeft[1] + robotTopRight[1]) / 2;

        // Calculates the angle between the robot and the destination using trig.
        float robotAngle = this.robotObject.getRotation();
        float xCoordinate = (x - robotCenterX);
        float yCoordinate = (y - robotCenterY);

        // Gets the angle of the robot +90 and finds the cosine and sine of that angle
        float rotateDegrees = robotAngle + 90;
        float c = toFloat(Math.cos(Math.toRadians(rotateDegrees)));
        float s = toFloat(Math.sin(Math.toRadians(rotateDegrees)));

        // Rotates the points to get the movement.
        float xInches = toFloat(xCoordinate*c - yCoordinate*s);
        float yInches = toFloat(xCoordinate*s - yCoordinate*c);
        // Adjust for real world application
        xInches *= 1.33;
        yInches *= 1.04;

        // Creates x and y matrices to be used to find how far each wheel should travel.
        Matrix xMatrix = new Matrix(2, 2);
        float[] driveX = {1, -1, -1, 1};
        xMatrix.setValues(driveX);
        Matrix yMatrix = new Matrix(2, 2);
        float[] driveY = {1, 1, 1, 1};
        yMatrix.setValues(driveY);

        // Scales the unit matrices by how far it needs to travel in the x and y axis.
        xMatrix.scalar(xInches);
        yMatrix.scalar(yInches);
        // MatrixHandler used to combine the x and y matrices into a final matrix (fin).
        MatrixHandler handler = new MatrixHandler(xMatrix, yMatrix);
        Matrix xy = handler.addMatrices();
        Matrix fin = xy;

        // Adds in the z matrix if the rotation degrees is not zero, meaning we want the robot to rotate.
        if (rotationDegrees != 0) {
            Matrix zMatrix = new Matrix(2, 2);
            float deg = rotationDegrees / 90;
            float dist = deg * 18.35f;
            float[] driveZ = {1, -1, 1, -1};
            zMatrix.setValues(driveZ);
            zMatrix.scalar(dist);
            MatrixHandler xyzHandler = new MatrixHandler(xy, zMatrix);
            fin = xyzHandler.addMatrices();
        }

        // Updates the robot's position to where it will be travelling to.
        robotObject.addRotation(rotationDegrees);
        robotObject.setX(x - 9);
        robotObject.setY(y - 9);

        // Float array full of telemetry data to output to telemetry. Used for debugging.
        float[] telemetryData = {x, y, xInches, yInches, robotX, robotY, /*radius, theta,*/ xCoordinate, yCoordinate};

        // Creates Data object to return.
        return new Data(telemetryData, fin);
    }

    /**
     * Takes a matrix and drives the robot using it. Shows the encoder positions and targets in telemetry.
     *
     * @param fin {@link Matrix} used to tell how far each wheel should travel. 2x2 only.
     * @return Array of ints containing the four targets for each wheel.
     */
    public int[] drive(Matrix fin) {
        // Determine new target position, and pass to motor controller
        int frontLeftTarget = robot.getCurrentPosition(Robot.Motor.FRONT_LEFT) + (int) (fin.getValue(0, 0) * COUNTS_PER_INCH);
        int frontRightTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int) (fin.getValue(1, 0) * COUNTS_PER_INCH);
        int backLeftTarget = robot.getCurrentPosition(Robot.Motor.BACK_LEFT) + (int) (fin.getValue(0, 1) * COUNTS_PER_INCH);
        int backRightTarget = robot.getCurrentPosition(Robot.Motor.BACK_RIGHT) + (int) (fin.getValue(1, 1) * COUNTS_PER_INCH);
        robot.setTargetPosition(Robot.Motor.FRONT_LEFT, frontLeftTarget);
        robot.setTargetPosition(Robot.Motor.FRONT_RIGHT, frontRightTarget);
        robot.setTargetPosition(Robot.Motor.BACK_LEFT, backLeftTarget);
        robot.setTargetPosition(Robot.Motor.BACK_RIGHT, backRightTarget);

        // Turn On RUN_TO_POSITION
        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion.
        runtime.reset();
        robot.setPower(Robot.Motor.FRONT_LEFT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.FRONT_RIGHT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.BACK_LEFT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.BACK_RIGHT, DRIVE_SPEED);

        // Return target array for tick method.
        return new int[] {frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget};
    }

    /**
     * Method to be looped, used to update telemetry and check if the robot has finished moving.
     *
     * @param telemetry {@link Telemetry} object to write the telemetryData to telemetry.
     * @param targets Array of integers containing the targets for each wheel to travel to.
     * @param telemetryData Array of floats to be outputted to telemetry. Used for debugging.
     */
    public void tick(Telemetry telemetry, int[] targets, float[] telemetryData) {
        // Gets variables from the targets array.
        int frontLeftTarget = targets[0];
        int frontRightTarget = targets[1];
        int backLeftTarget = targets[2];
        int backRightTarget = targets[3];

        // Display it for the driver.
        telemetry.addData( "Running to", "FL %7d : FR %7d : BL %7d : BR %7d",
                frontLeftTarget,
                frontRightTarget,
                backLeftTarget,
                backRightTarget);
        telemetry.addData("Starting at", "FL %7d : FR %7d : BL %7d : BR %7d",
                robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));

        // Loops through telemetryData to display.
        for (int i = 0; i < telemetryData.length; i++) {
            telemetry.addData("data" + i, telemetryData[i]);
        }

        // Updates telemetry.
        telemetry.update();
    }

    /**
     * Stop method stops all power to the motors and turns off encoders.
     */
    public void stop() {
        // Stop all motion;
        robot.setPower(Robot.Motor.FRONT_LEFT, 0);
        robot.setPower(Robot.Motor.FRONT_RIGHT, 0);
        robot.setPower(Robot.Motor.BACK_LEFT, 0);
        robot.setPower(Robot.Motor.BACK_RIGHT, 0);

        // Turn off RUN_TO_POSITION
        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Function to pick up a block into the intake slot.
     */
    public void pickUpBlock() {
        // Starts driving forward.
        robot.setPower(Robot.Motor.FRONT_LEFT, 1);
        robot.setPower(Robot.Motor.FRONT_RIGHT, 1);
        robot.setPower(Robot.Motor.BACK_LEFT, 1);
        robot.setPower(Robot.Motor.BACK_RIGHT, 1);

        // Starts wheel intake.
        robot.setWheelIntake(1);
    }

    /**
     * Function to drop a block from the intake slot.
     */
    public void dropBlock() {
        // Starts driving backwards.
        robot.setPower(Robot.Motor.FRONT_LEFT, -1);
        robot.setPower(Robot.Motor.FRONT_RIGHT, -1);
        robot.setPower(Robot.Motor.BACK_LEFT, -1);
        robot.setPower(Robot.Motor.BACK_RIGHT, -1);

        // Starts intake in reverse.
        robot.setWheelIntake(-1);
    }

    /**
     * Reverts back to its original position. Essentially an inverse pick up/drop of the block without running the intake.
     *
     * @param par0: A string to tell it what to revert, either "grab" or "drop".
     */
    public void revert(String par0) {
        if (par0.equals("grab")) {
            robot.setPower(Robot.Motor.FRONT_LEFT, -1);
            robot.setPower(Robot.Motor.FRONT_RIGHT, -1);
            robot.setPower(Robot.Motor.BACK_LEFT, -1);
            robot.setPower(Robot.Motor.BACK_RIGHT, -1);
        } else if (par0.equals("drop")) {
            robot.setPower(Robot.Motor.FRONT_LEFT, 1);
            robot.setPower(Robot.Motor.FRONT_RIGHT, 1);
            robot.setPower(Robot.Motor.BACK_LEFT, 1);
            robot.setPower(Robot.Motor.BACK_RIGHT, 1);
        }
    }

    /**
     * Turns a double into a float.
     *
     * @param a: Double to be transformed.
     * @return Returns a float with the same value as the double.
     */
    public static float toFloat(double a) {
        return Float.valueOf(String.valueOf(a));
    }

    /**
     * Data class to contain both a {@link Matrix} and a float array for telemetry.
     */
    public class Data {

        public float[] telemetryData;
        public Matrix fin;

        Data(float[] telemetryData, Matrix fin) {
            this.telemetryData = telemetryData;
            this.fin = fin;
        }
    }
}
