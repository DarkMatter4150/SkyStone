package TutorialCodeWithDocs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.breakout.BreakoutMotor;

import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_F;

/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
//@TeleOp(name = "Demo", group = "Pushbot")

public class DemoDriveOp extends OpMode {
    //Motor objects
    private BreakoutMotor motorLeft = new BreakoutMotor();
    private BreakoutMotor motorRight = new BreakoutMotor();
    private BreakoutMotor motorStrafe = new BreakoutMotor();

    @Override
    public void init() {
        //Broken out motor class
        //Set hardwaremaps for left and right motors
        motorLeft.set(hardwareMap.dcMotor.get("motorLeft"));
        motorRight.set(hardwareMap.dcMotor.get("motorRight"));
        motorStrafe.set(hardwareMap.dcMotor.get("motorStrafe"));
        //Set directions for left and right motors
        //F = Clockwise while looking at axle
        //R = Counter clockwise while looking at axle
        motorLeft.setDirection(MOTOR_F);
        motorRight.setDirection(MOTOR_F);
        motorStrafe.setDirection(MOTOR_F);
        //Start with 0 power for both motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorStrafe.setPower(0);

        //Clear telemetry
        telemetry.clearAll();
    }

    @Override
    public void start() {
        //Motor start
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorStrafe.setPower(0);
    }

    @Override
    public void loop() {

        //Y+ : forward, Y- : backwards
        //X+ : right, X- : Left

        //Gamepad 1
        float leftStick1x = gamepad1.left_stick_x;
        float leftStick1y = -gamepad1.left_stick_y;
        float rightStick1x = gamepad1.right_stick_x;
        float rightStick1y = -gamepad1.right_stick_y;
        float leftTrigger1 = gamepad1.left_trigger;
        float rightTrigger1 = gamepad1.right_trigger;
        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = -gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = -gamepad2.right_stick_y;

        //Move the motors//
        float power = 0;
        float turnpower = 0;
        if (leftStick1y > 0) {
            power = leftStick1y;
            if (rightStick1x > 0) {
                turnpower = rightStick1x;
                motorLeft.setPower(power - turnpower);
                motorRight.setPower(power);
            } else if (rightStick1x < 0) {
                turnpower = rightStick1x;
                motorLeft.setPower(power);
                motorRight.setPower(power + turnpower);
            } else {
                motorLeft.setPower(power);
                motorRight.setPower(power);
            }
        } else if (leftStick1y < 0) {
            power = leftStick1y;
            if (rightStick1x > 0) {
                turnpower = rightStick1x;
                motorLeft.setPower(power + turnpower);
                motorRight.setPower(power);
            } else if (rightStick1x < 0) {
                turnpower = rightStick1x;
                motorLeft.setPower(power);
                motorRight.setPower(power - turnpower);
            } else {
                motorLeft.setPower(power);
                motorRight.setPower(power);
            }
        } else {
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }

        motorStrafe.setPower(rightTrigger1-leftTrigger1);

        //Telemetry
        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
        telemetry.addData("Right Stick Y 1", rightStick1y);
        telemetry.addData("Left Stick X 2", leftStick2x);
        telemetry.addData("Left Stick Y 2", leftStick2y);
        telemetry.addData("Power", power);
        telemetry.addData("TurnPower", turnpower);
    }

    @Override
    public void stop() {
        //Motor stop
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
