package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

//@Autonomous(name="first comp auto MOVES RIGHT", group="basic")
public class FirstCompAuto extends LinearOpMode {

    private Robot robot = new Robot(telemetry);
    private Mecanum drive = new Mecanum(robot, telemetry);
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        //3 feet per second of drive @ speed = 1
        double startTime = timer.milliseconds();
        drive.setPower(1, 1, 0);
        while (timer.milliseconds() - startTime < 1000);
        drive.setPower( 0, 0, 0);
    }

    public void moveFoundation() {
        double startTime = timer.milliseconds();
        drive.setPower(0, -1, 0);
        while (timer.milliseconds() - startTime < 1333.333);
        drive.setPower(0,0,0);
        robot.setTabs(false);
        drive.setPower(0, 1, 0);
    }
}
