package org.firstinspires.ftc.teamcode.breakout;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_R;

public class Robot {

    public enum Motor {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }

    public enum ClawPos {
        OPEN(0.83d), CLOSED(1.0d);

        private double pos;

        ClawPos(double pos) { this.pos = pos; }
    }

    public enum TabPos {
        LEFT_OPEN(1.0d), LEFT_CLOSED(0.56d),
        RIGHT_OPEN(0.08d), RIGHT_CLOSED(0.52d);

        private double pos;

        TabPos(double pos) { this.pos = pos; }
    }

    //Motors
    private BreakoutMotor frontLeft = new BreakoutMotor();
    private BreakoutMotor frontRight = new BreakoutMotor();
    private BreakoutMotor backLeft = new BreakoutMotor();
    private BreakoutMotor backRight = new BreakoutMotor();
    private BreakoutMotor wheelIntakeLeft = new BreakoutMotor();
    private BreakoutMotor wheelIntakeRight = new BreakoutMotor();
    private BreakoutMotor arm = new BreakoutMotor();

    //Servos
    private BreakoutServo tabLeft = new BreakoutServo();
    private BreakoutServo tabRight = new BreakoutServo();
    private BreakoutServo claw = new BreakoutServo();

    //Gyro
    private BreakoutREVGyro gyro = new BreakoutREVGyro();
    private Orientation lastAngles = new Orientation();
    private double globalAngle = 0;
    private double targetAngle = 0;

    //Misc
    private HardwareMap hardwareMap;
    private ElapsedTime period = new ElapsedTime();
    private Telemetry telemetry;
    private double startTime;
    private double lastTime = -1;
    double previousError = 0;
    private final double ARM_CYCLES_PER_REV = 1425.2;

    /* Constructor */
    public Robot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public String tag(String driveOp) {
        return "DM4150 " + driveOp + ": ";
    }

    public void setWheelIntake(float power) {
        wheelIntakeLeft.setPower(-power);
        wheelIntakeRight.setPower(power);
    }

    public void setTabs(boolean open) {
        if (open) {
            tabLeft.setPosition(TabPos.LEFT_OPEN.pos);
            tabRight.setPosition(TabPos.RIGHT_OPEN.pos);
        } else {
            tabLeft.setPosition(TabPos.LEFT_CLOSED.pos);
            tabRight.setPosition(TabPos.RIGHT_CLOSED.pos);
        }
    }

    public void setClaw(boolean open) {
        if (open) {
            claw.setPosition(ClawPos.OPEN.pos);
        } else {
            claw.setPosition(ClawPos.CLOSED.pos);
        }
    }

    public double getClawPos() {
        return claw.getPosition();
    }

    @Deprecated
    public void setArmPower(float power) {
        arm.setPower(power);
    }

    public void moveArm(float power) {
        arm.setTargetPosition(arm.getCurrentPosition() + (int)(power*80));
        arm.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (arm.getCurrentPosition() + (int)(power*104) < arm.getCurrentPosition()) {
            arm.setPower(-1);
        } else {
            arm.setPower(1);
        }
    }

    public void setPower(Motor motor, float power) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setPower(power);
                break;
            case FRONT_RIGHT:
                frontRight.setPower(power);
                break;
            case BACK_LEFT:
                backLeft.setPower(power);
                break;
            case BACK_RIGHT:
                backRight.setPower(power);
                break;
        }
    }

    public double getPower(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.getPower();
            case FRONT_RIGHT:
                return frontRight.getPower();
            case BACK_LEFT:
                return backLeft.getPower();
            case BACK_RIGHT:
                return backRight.getPower();
            default:
                return 0;
        }
    }

    public void setRunMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setMotorMode(mode);
                break;
            case FRONT_RIGHT:
                frontRight.setMotorMode(mode);
                break;
            case BACK_LEFT:
                backLeft.setMotorMode(mode);
                break;
            case BACK_RIGHT:
                backRight.setMotorMode(mode);
                break;
        }
    }

    public void setTargetPosition(Motor motor, int pos) {
        switch (motor) {
            case FRONT_LEFT:
                frontLeft.setTargetPosition(pos);
                break;
            case FRONT_RIGHT:
                frontRight.setTargetPosition(pos);
                break;
            case BACK_LEFT:
                backLeft.setTargetPosition(pos);
                break;
            case BACK_RIGHT:
                backRight.setTargetPosition(pos);
                break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.isBusy();
            case FRONT_RIGHT:
                return frontRight.isBusy();
            case BACK_LEFT:
                return backLeft.isBusy();
            case BACK_RIGHT:
                return backRight.isBusy();
            default:
                return false;
        }
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case FRONT_LEFT:
                return frontLeft.getCurrentPosition();
            case FRONT_RIGHT:
                return frontRight.getCurrentPosition();
            case BACK_LEFT:
                return backLeft.getCurrentPosition();
            case BACK_RIGHT:
                return backRight.getCurrentPosition();
            default:
                return 8008135;
        }
    }

    public Orientation getAngularOrientation() {
        return gyro.getOrient(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
    }

    public void resetAngle() {
        targetAngle = gyro.getOrient(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("target", targetAngle);
    }

    private double getAngle() {
        Orientation current = getAngularOrientation();

        double angleDif = targetAngle - current.firstAngle;
        double invertDirectionAngle = targetAngle - 180;

        if (angleDif < -180) {
            angleDif = 360 + angleDif;
        }
        if (angleDif > 180) {
            angleDif = -360 + angleDif;
        }

        telemetry.addData("lastangles", lastAngles.firstAngle);
        telemetry.addData("angleDif", angleDif);

        return angleDif;
    }

    public double checkDirection() {

        if (lastTime == -1) {
            lastTime = startTime - 1;
        }

        double timeDif = period.milliseconds() - lastTime;

        double correction, angle;

//        double Ku = 3.16;
        double Kp = -0.00128;
        double Ki = 0;
        double Kd = 0.01;

        angle = getAngle();

        if (angle > 360 || angle < -360) {
            resetAngle();
            angle = 0;
        }

        double p = Kp * angle;
        double i = Ki * (angle * timeDif);
        double d = Kd * (angle - previousError) / timeDif;

        previousError = angle;

        correction = p + i + d;
        telemetry.addData("target", targetAngle);
        telemetry.addData("Correction", correction);
        telemetry.addData("Global Angle", angle);
        return correction;
    }

    public void start() {
        this.startTime = period.milliseconds();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Gyro init
        gyro.set(hardwareMap.get(gyro.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
        telemetry.update();
        gyro.calibrate();
        telemetry.clearAll();

        // Define and Initialize Motors
        frontLeft.set(hardwareMap.dcMotor.get("frontLeft"));
        frontRight.set(hardwareMap.dcMotor.get("frontRight"));
        backLeft.set(hardwareMap.dcMotor.get("backLeft"));
        backRight.set(hardwareMap.dcMotor.get("backRight"));

        wheelIntakeLeft.set(hardwareMap.dcMotor.get("leftIntake"));
        wheelIntakeRight.set(hardwareMap.dcMotor.get("rightIntake"));

        tabLeft.set(hardwareMap.servo.get("tabLeft"));
        tabRight.set(hardwareMap.servo.get("tabRight"));

        arm.set(hardwareMap.dcMotor.get("arm"));
        claw.set(hardwareMap.servo.get("claw"));

        //Set directions for left and right motors
        //F = Clockwise while looking at axle
        //R = Counter clockwise while looking at axle
        frontLeft.setDirection(MOTOR_R);
        frontRight.setDirection(MOTOR_F);
        backLeft.setDirection(MOTOR_R);
        backRight.setDirection(MOTOR_F);

        wheelIntakeLeft.setDirection(MOTOR_F);
        wheelIntakeRight.setDirection(MOTOR_R);

        arm.setDirection(MOTOR_F);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        wheelIntakeLeft.setPower(0);
        wheelIntakeRight.setPower(0);

        tabLeft.setPosition(TabPos.LEFT_OPEN.pos);
        tabRight.setPosition(TabPos.RIGHT_OPEN.pos);

        arm.setPower(0);
        claw.setPosition(ClawPos.OPEN.pos);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelIntakeLeft.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelIntakeRight.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
