package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

//@Autonomous(name="Blue Autonomous", group="pushbot")
public class DeprecatedAutoBlue extends LinearOpMode {

    private Robot robot = new Robot(telemetry);
    private Mecanum drive = new Mecanum(robot, telemetry);
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

//        moveFoundation();
//        foundation();
//        //3 feet per second of drive @ speed = 1
        double startTime = timer.milliseconds();
        drive.setPower(1, 0, 0);
        while (timer.milliseconds() - startTime < 1000);
        drive.setPower( 0, 0, 0);
    }

    public void foundation() {
        drive.setPower(0, 1, 0);
        double time = timer.milliseconds();
        while (timer.milliseconds() - time < 2000 && opModeIsActive());
        drive.setPower(-1, 0, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 1200 && opModeIsActive());
        drive.setPower(0, 0, 0);
        robot.setTabs(false);
        drive.setPower(0, -1, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 1750 && opModeIsActive());
        drive.setPower(0, 1, 0);
        robot.setTabs(true);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 200 && opModeIsActive());
        drive.setPower(1, 0, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 2500 && opModeIsActive());
        drive.setPower(0, 0, 0);
        stop();
    }

    public void moveFoundation() {
        drive.setPower(1, 0, -0.1f);
        double time = timer.milliseconds();
        while (timer.milliseconds() - time < 450 && opModeIsActive());
        drive.setPower(0, 0, 0);
        double startTime = timer.milliseconds();
        drive.setPower(0, -1, 0);
        while (timer.milliseconds() - startTime < 1500 && opModeIsActive());
        drive.setPower(0,0,0);
        robot.setTabs(false);
        drive.setPower(0, 1, 0);
        while (timer.milliseconds() - startTime < (2000) && opModeIsActive());
        drive.setPower(0, 0, 0);
        robot.setTabs(true);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < (100) && opModeIsActive());
        drive.setPower(0, 1, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < (500) && opModeIsActive());
        drive.setPower(-1, 0, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < (450) && opModeIsActive());
        drive.setPower(0, 0, 1);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < (1200) && opModeIsActive());
    }
}
