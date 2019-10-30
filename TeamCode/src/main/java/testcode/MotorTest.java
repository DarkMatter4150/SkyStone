package testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.breakout.BreakoutMotor;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends OpMode {
    private BreakoutMotor frontLeft = new BreakoutMotor();

    @Override
    public void init() {
        frontLeft.set(hardwareMap.dcMotor.get("frontLeft"));

        frontLeft.setDirection(BreakoutMotor.Direction.MOTOR_F);

        frontLeft.setPower(0);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLeft.setPower(0.25);
        } else if (gamepad1.b) {
            frontLeft.setPower(0.5);
        } else if (gamepad1.x) {
            frontLeft.setPower(0.75);
        } else if (gamepad1.y) {
            frontLeft.setPower(1);
        } else {
            frontLeft.setPower(0);
        }

        telemetry.addData("A (0.25)", gamepad1.a);
        telemetry.addData("B (0.5)", gamepad1.b);
        telemetry.addData("X (0.75)", gamepad1.x);
        telemetry.addData("Y (1)", gamepad1.y);

        telemetry.update();
    }
}
