package ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Drive", group = "Pushbot")

public class DriveOp extends OpMode {

    private Wheels wheels;

    private double elePos = 2;

    public DriveOp() {
        wheels = new Wheels();
    }

    @Override
    public void init() {
        DcMotor motorLeft = hardwareMap.dcMotor.get("MotorLeft");
        DcMotor motorRight = hardwareMap.dcMotor.get("MotorRight");
        DcMotor motorElevator = hardwareMap.dcMotor.get("ElevatorMotor");
        Servo servoLeft = hardwareMap.servo.get("ServoLeft");
        Servo servoRight = hardwareMap.servo.get("ServoRight");
        wheels.init(motorLeft, motorRight, motorElevator, servoLeft, servoRight);
    }

    @Override
    public void start() {
        wheels.start();
    }

    @Override
    public void loop() {
        // Gamepad sticks are inverted.

        float leftStick1 = gamepad1.left_stick_y;
        float rightStick1 = gamepad1.right_stick_y;
        float leftStick2 = gamepad2.left_stick_y;
        float rightStick2 = Math.abs(gamepad2.right_stick_y);

        wheels.move(-leftStick1, -rightStick1, -leftStick2, rightStick2);

        telemetry.addData("Left Y", leftStick1);
        telemetry.addData("Right Y", rightStick1);
        telemetry.addData("Elevator Y", leftStick2);
        telemetry.addData("Servo Y", rightStick2);
    }

    @Override
    public void stop() {
        wheels.stop();
    }
}




