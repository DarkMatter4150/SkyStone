package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

@TeleOp(name="pls help", group="test")
public class MotorConfusionTest extends OpMode {

    private Robot robot = new Robot(telemetry);
    private ElapsedTime timer = new ElapsedTime();
    private Mecanum drive;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.resetAngle();

        //Clear telemetry
        telemetry.clearAll();

        //Mecanum drive handler
        drive = new Mecanum(robot, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            drive.setPower(0, 1, 0);
        } else if (gamepad1.b) {
            drive.setPower(0, 0.3f, 0);
        } else if (gamepad1.x) {
            drive.setPower(0, 0.2f, 0);
        } else if (gamepad1.y) {
            drive.setPower(0, 0.1f, 0);
        } else {
            drive.setPower(0, 0, 0);
        }
    }

    @Override
    public void stop() {
        drive.setPower(0, 0, 0);
    }
}
