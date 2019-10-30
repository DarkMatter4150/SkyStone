package testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "DistanceTest", group = "Pushbot")

public class DistanceTest extends OpMode {

    private DcMotor leftSide;
    private DcMotor rightSide;


    private double Speed = 0.4;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

    private void setLeftPower(double pwr) {
        double leftPower = Range.clip(pwr, -1, 1);
        leftSide.setPower(leftPower);
    }

    private void setRightPower(double pwr) {
        double rightPower = Range.clip(pwr, -1, 1);
        rightSide.setPower(rightPower);
    }

    private void setPower(double l, double r) {
        setLeftPower(l);
        setRightPower(r);
    }

    private void move() {
        setPower(Speed, Speed);
    }

    private void setWheelMode(DcMotor.RunMode mode) {
        leftSide.setMode(mode);
        rightSide.setMode(mode);
    }

    @Override
    public void stop() {
        setPower(0, 0);
    }

    @Override
    public void start() {
        setPower(0.0, 0.0);
        timer.reset();
        timer2.reset();
        telemetry.addData("Start", "");
    }

    private void test() {

        telemetry.addData("before reset", "");

        timer.reset();
        timer2.reset();

        move();

        telemetry.addData("after reset", "");

        while (timer.time() < 1) {

            telemetry.addData("Time", timer.time());
            telemetry.addData("Time2", timer2.time());

        }

        stop();

    }

    @Override
    public void init() {
        leftSide = hardwareMap.dcMotor.get("MotorLeft");
        rightSide = hardwareMap.dcMotor.get("MotorRight");

        leftSide.setDirection(DcMotor.Direction.REVERSE);
        rightSide.setDirection(DcMotor.Direction.FORWARD);

        setWheelMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Init", "");

        test();
    }

    @Override
    public void loop() {

    }
}
