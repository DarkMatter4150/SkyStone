package ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import ExampleCode.BreakoutMotor;
import BreakoutBase.BreakoutREVGyro;
import BreakoutBase.BreakoutServo;

import static ExampleCode.BreakoutMotor.Direction.MOTOR_F;
import static ExampleCode.BreakoutMotor.Direction.MOTOR_R;
import static ExampleCode.BreakoutServo.Direction.SERVO_F;
import static ExampleCode.BreakoutServo.Direction.SERVO_R;

/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
//@TeleOp(name = "BreakoutDrive", group = "Pushbot")

public class BreakoutDriveOp extends OpMode {

    //Servo Breakout code definition
    private BreakoutServo servoA = new BreakoutServo();
    private BreakoutServo servoB = new BreakoutServo();

    //Motor Breakout code definition
    private BreakoutMotor motorA = new BreakoutMotor();
    private BreakoutMotor motorB = new BreakoutMotor();

    //Gyro Breakout code definition
    private BreakoutREVGyro gyroA = new BreakoutREVGyro();

    //Timer definition
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {

        //Broken out servo class
        servoA.set(hardwareMap.servo.get("servoA"));
        servoB.set(hardwareMap.servo.get("servoB"));
//        servoA.setDirection(SERVO_F);
//        servoB.setDirection(SERVO_R);
        servoA.setPosition(0);
        servoB.setPosition(0);

        //Broken out motor class
        motorA.set(hardwareMap.dcMotor.get("motorA"));
        motorB.set(hardwareMap.dcMotor.get("motorB"));
        motorA.setDirection(MOTOR_F);
        motorB.setDirection(MOTOR_R);
        motorA.setPower(0);
        motorB.setPower(0);

        //Broken out Gyro class
        gyroA.set(hardwareMap.get(gyroA.IMU, "gyroA"));
        telemetry.addLine("Calibrating: DO NOT MOVE!");
//        gyroA.calibrate();
        telemetry.clearAll();

    }

    @Override
    public void start() {

        //Motor start
        motorA.setPower(0);
        motorB.setPower(0);

        //Servo start
        servoA.setPosition(0);
        servoB.setPosition(0);

    }

    @Override
    public void loop() {

        //Gamepad 1
        float leftStick1x = gamepad1.left_stick_x;
        float leftStick1y = gamepad1.left_stick_y;
        float rightStick1x = gamepad1.right_stick_x;
        float rightStick1y = gamepad1.right_stick_y;
        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = gamepad2.right_stick_y;

        //Move the motors
        motorA.setPower(-leftStick1y);
        motorB.setPower(-rightStick1y);

        //Set the servos
        servoA.setPosition(leftStick2y);
        servoB.setPosition(rightStick1y);

        //Telemetry
        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
        telemetry.addData("Right Stick Y 1", rightStick1y);
        telemetry.addData("Left Stick X 2", leftStick2x);
        telemetry.addData("Left Stick Y 2", leftStick2y);
        telemetry.addData("Right Stick X 2", rightStick2x);
        telemetry.addData("Right Stick Y 2", rightStick2y);
        telemetry.addData("Gyro Heading", gyroA.getPos());
        telemetry.addData("Gyro Orientation", gyroA.getOrient());
        telemetry.addData("Gyro Velocity", gyroA.getVel());
        telemetry.addData("Gyro Acceleration", gyroA.getAccel());
    }

    @Override
    public void stop() {

        //Motor stop
        motorA.setPower(0);
        motorB.setPower(0);

        //Servo stop
        servoA.setPosition(0);
        servoB.setPosition(0);

    }

}
