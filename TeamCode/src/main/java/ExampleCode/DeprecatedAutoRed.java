package ExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

//@Autonomous(name="Red Autonomous", group="pushbot")
public class DeprecatedAutoRed extends LinearOpMode {

    private Robot robot = new Robot(telemetry);
    private Mecanum drive = new Mecanum(robot, telemetry);
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

//        foundation();
        //3 feet per second of drive @ speed = 1
        double startTime = timer.milliseconds();
        drive.setPower(-1, 0, 0);
        while (timer.milliseconds() - startTime < 1000);
        drive.setPower( 0, 0, 0);
    }

    public void foundation() {
        drive.setPower(0, 1, 0);
        double time = timer.milliseconds();
        while (timer.milliseconds() - time < 2300 && opModeIsActive());
        drive.setPower(1, 0, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 1200 && opModeIsActive());
        drive.setPower(0, 0, 0);
        robot.setTabs(true);
        drive.setPower(0, -1, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 2300 && opModeIsActive());
        drive.setPower(0, 1, 0);
        robot.setTabs(false);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 200 && opModeIsActive());
        drive.setPower(-1, 0, 0);
        time = timer.milliseconds();
        while (timer.milliseconds() - time < 2500 && opModeIsActive());
        drive.setPower(0, 0, 0);
        stop();
    }

    @Deprecated
    public void moveFoundation() {
        drive.setPower(-1, 0, -0.1f);
        double time = timer.milliseconds();
        while (timer.milliseconds() - time < 450 && opModeIsActive());
        drive.setPower(0, 0, 0);
        double startTime = timer.milliseconds();
        drive.setPower(0, -1, 0);
        while (timer.milliseconds() - startTime < 1500 && opModeIsActive());
        drive.setPower(0,0,0);
        robot.setTabs(false);
        drive.setPower(0, 1, 0);
        while (timer.milliseconds() - startTime < (3500) && opModeIsActive());
        drive.setPower(0, 0, 0);
        robot.setTabs(true);
        while (timer.milliseconds() - startTime < (5000) && opModeIsActive());
    }
}
