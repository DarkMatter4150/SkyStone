package testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "Pushbot")

public class ServoTest extends OpMode {

    private Servo servTest;

    @Override
    public void init() {
        servTest = hardwareMap.servo.get("ServoTest");
        servTest.setDirection(Servo.Direction.FORWARD);
    }

    public void start() {
        servTest.setPosition(0.0);
    }

    @Override
    public void loop() {

        servTest.setPosition(Math.abs(gamepad1.left_stick_y));

    }

    public void stop() {
        servTest.setPosition(0.0);
    }

}
