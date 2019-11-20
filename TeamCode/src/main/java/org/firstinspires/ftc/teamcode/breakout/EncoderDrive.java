package org.firstinspires.ftc.teamcode.breakout;

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
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final float DRIVE_SPEED = 1f;

    public static final double TAU = 2 * Math.PI;

    public EncoderDrive(Robot robot, RobotObject robotObject) {
        this.robot = robot;
        this.robotObject = robotObject;
    }

    public Data getDriveMatrix(FieldObject destination,
                                 float xOffset, float yOffset, float rotationDegrees) {

        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_USING_ENCODER);

        float destL = destination.getLength();
        float destW = destination.getWidth();
        float destX = destination.getX();
        float destY = destination.getY();
        float[] destTopLeft = {destX, destY + destL};
        float[] destTopRight = {destX + destW, destY + destL};
        float[] destBottomLeft = {destX, destY};
        float[] destBottomRight = {destX + destW, destY};

        float robotL = robotObject.getLength();
        float robotW = robotObject.getWidth();
        float robotX = robotObject.getX();
        float robotY = robotObject.getY();
        float[] robotTopLeft = {robotX, robotY + robotL};
        float[] robotTopRight = {robotX + robotW, robotY + robotL};
        float[] robotBottomLeft = {robotX, robotY};
        float[] robotBottomRight = {robotX + robotW, robotY};

        float destCenterX = (destBottomLeft[0] + destTopRight[0]) / 2;
        float destCenterY = (destBottomLeft[1] + destTopRight[1]) / 2;

        float robotCenterX = (robotBottomLeft[0] + robotTopRight[0]) / 2;
        float robotCenterY = (robotBottomLeft[1] + robotTopRight[1]) / 2;
        float[] robotCenter = {robotCenterX, robotCenterY};
        float[] destCenter = {destCenterX, destCenterY};

        float robotangle = this.robotObject.getRotation();
        double destangle = Math.atan((destBottomLeft[1] - robotBottomLeft[1]) / (destBottomLeft[0] - robotBottomLeft[0]));
        double theta = Math.toRadians(destangle - robotangle + 90);
        double c = Math.cos(theta);
        double s = Math.sin(theta);

        float xInRelation1 = destBottomLeft[0] - robotCenter[0];
        float yInRelation1 = destBottomLeft[1] - robotCenter[1];
        float xInRelation2 = destBottomRight[0] - robotCenter[0];
        float yInRelation2 = destBottomRight[1] - robotCenter[1];
        float xInRelation3 = destTopLeft[0] - robotCenter[0];
        float yInRelation3 = destTopLeft[1] - robotCenter[1];
        float xInRelation4 = destTopRight[0] - robotCenter[0];
        float yInRelation4 = destTopRight[1] - robotCenter[1];

        float newX = Float.valueOf(String.valueOf((xInRelation1 * c) - (yInRelation1 * s))) + robotCenter[0];
        float newY = Float.valueOf(String.valueOf((yInRelation1 * s) + (xInRelation1 * c))) + robotCenter[1];

        destTopLeft[0] = Float.valueOf(String.valueOf((xInRelation3 * c) - (yInRelation3 * s))) + robotCenter[0];
        destTopLeft[1] = Float.valueOf(String.valueOf((yInRelation3 * s) + (xInRelation3 * c))) + robotCenter[1];

        destTopRight[0] = Float.valueOf(String.valueOf((yInRelation2 * c) - (xInRelation2 * s))) + robotCenter[1];
        destTopRight[1] = Float.valueOf(String.valueOf((yInRelation2 * s) + (xInRelation2 * c))) + robotCenter[1];

        destBottomLeft[0] = newX;
        destBottomLeft[1] = newY;

        destBottomRight[0] = Float.valueOf(String.valueOf((yInRelation4 * c) - (xInRelation4 * s))) + robotCenter[1];
        destBottomRight[1] = Float.valueOf(String.valueOf((yInRelation4 * s) + (xInRelation4 * c))) + robotCenter[1];
        
//        boolean robotIsLeftOfDest = false;
//        boolean robotIsAboveDest = false;
//        if (!((robotTopLeft[0] + robotTopRight[0]) / 2 < (destTopLeft[0] + destTopRight[0]) / 2)) {
//            robotIsLeftOfDest = true;
//        }
//        if (!((robotTopLeft[1] + robotBottomLeft[1]) / 2 < (destTopLeft[1] + destBottomLeft[1]) / 2)) {
//            robotIsAboveDest = true;
//        }
//
//        if (robotIsAboveDest) {
//            yInches = -(robotBottomRight[1] - destTopRight[1]) + yOffset;
//        } else {
//            yInches = destBottomRight[1] - robotTopRight[1] - yOffset;
//        }
//        if (robotIsLeftOfDest) {
//            xInches = destBottomLeft[0] - robotBottomRight[0] + xOffset;
//        } else {
//            xInches = -(robotBottomLeft[0] - destBottomRight[0]) - xOffset;
//        }

        //TODO: test this code, and figure out offsets

        float gotoX = destCenterX + xOffset;
        float gotoY = destCenterY + yOffset;

        float xInches = gotoX - robotCenterX;
        float yInches = gotoY - robotCenterY;

        float[] teledata = {gotoX, gotoY, robotBottomLeft[0], robotBottomLeft[1], Float.valueOf(String.valueOf(Math.toDegrees(theta)))};

        //TODO: figure out rotating (done i think, just test)
        //formula for arc length (degrees): theta/360 * tau * radius
        float zInches = (float)( (rotationDegrees / 360) * (TAU * 25.4558) );

        Matrix xMatrix = new Matrix(2, 2);
        float[] driveX = {1, -1, -1, 1};
        xMatrix.setValues(driveX);
        Matrix yMatrix = new Matrix(2, 2);
        float[] driveY = {1, 1, 1, 1};
        yMatrix.setValues(driveY);

        xMatrix.scalar(xInches);
        yMatrix.scalar(yInches);
        MatrixHandler handler = new MatrixHandler(xMatrix, yMatrix);
        Matrix xy = handler.addMatrices();
        Matrix fin = xy;

        if (rotationDegrees != 0) {
            Matrix zMatrix = new Matrix(2, 2);
            float[] driveZ = {1, -1, 1, -1};
            zMatrix.setValues(driveZ);
            zMatrix.scalar(zInches);
            MatrixHandler xyzHandler = new MatrixHandler(xy, zMatrix);
            fin = xyzHandler.addMatrices();
        }

        robotObject.setX(destination.getX() + xOffset);
        robotObject.setY(destination.getY() + yOffset);

        return new Data(teledata, fin);
    }

    public int[] drive(Matrix fin) {

        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;

        // Determine new target position, and pass to motor controller
        frontLeftTarget = robot.getCurrentPosition(Robot.Motor.FRONT_LEFT) + (int) (fin.getValue(0, 0) * COUNTS_PER_INCH);
        frontRightTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int) (fin.getValue(1, 0) * COUNTS_PER_INCH);
        backLeftTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int) (fin.getValue(0, 1) * COUNTS_PER_INCH);
        backRightTarget = robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT) + (int) (fin.getValue(1, 1) * COUNTS_PER_INCH);
        robot.setTargetPosition(Robot.Motor.FRONT_LEFT, frontLeftTarget);
        robot.setTargetPosition(Robot.Motor.FRONT_RIGHT, frontRightTarget);
        robot.setTargetPosition(Robot.Motor.BACK_LEFT, backLeftTarget);
        robot.setTargetPosition(Robot.Motor.BACK_RIGHT, backRightTarget);

        // Turn On RUN_TO_POSITION
        robot.setRunMode(Robot.Motor.FRONT_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.FRONT_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.BACK_LEFT, DcMotor.RunMode.RUN_TO_POSITION);
        robot.setRunMode(Robot.Motor.BACK_RIGHT, DcMotor.RunMode.RUN_TO_POSITION);
        // reset the timeout time and start motion.
        runtime.reset();
        robot.setPower(Robot.Motor.FRONT_LEFT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.FRONT_RIGHT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.BACK_LEFT, DRIVE_SPEED);
        robot.setPower(Robot.Motor.BACK_RIGHT, DRIVE_SPEED);

        return new int[]{frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget};
    }

    public void tick(Telemetry telemetry, int[] targets, Matrix matrix, float[] teledata) {
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        int frontLeftTarget = targets[0];
        int frontRightTarget = targets[1];
        int backLeftTarget = targets[2];
        int backRightTarget = targets[3];

        // Display it for the driver.
        telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                frontLeftTarget,
                frontRightTarget,
                backLeftTarget,
                backRightTarget);
        telemetry.addData("Path2", "Starting at %7d : %7d : %7d : %7d",
                robot.getCurrentPosition(Robot.Motor.FRONT_LEFT),
                robot.getCurrentPosition(Robot.Motor.FRONT_RIGHT),
                robot.getCurrentPosition(Robot.Motor.BACK_LEFT),
                robot.getCurrentPosition(Robot.Motor.BACK_RIGHT));
        telemetry.addData("Drive", matrix.toString());

        for (int i = 0; i < teledata.length; i++) {
            telemetry.addData("data" + i, teledata[i]);
        }

        telemetry.update();
    }

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

    public void pickUpBlock() {
        robot.setPower(Robot.Motor.FRONT_LEFT, 1);
        robot.setPower(Robot.Motor.FRONT_RIGHT, 1);
        robot.setPower(Robot.Motor.BACK_LEFT, 1);
        robot.setPower(Robot.Motor.BACK_RIGHT, 1);

        robot.setWheelIntake(1);
    }

    public void dropBlock() {
        robot.setPower(Robot.Motor.FRONT_LEFT, -1);
        robot.setPower(Robot.Motor.FRONT_RIGHT, -1);
        robot.setPower(Robot.Motor.BACK_LEFT, -1);
        robot.setPower(Robot.Motor.BACK_RIGHT, -1);

        robot.setWheelIntake(-1);
    }

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

    public class Data {

        public float[] teledata;
        public Matrix fin;

        public Data(float[] teledata, Matrix fin) {
            this.teledata = teledata;
            this.fin = fin;
        }
    }
}
