package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.breakout.BreakoutMotor;
import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_F;
import static org.firstinspires.ftc.teamcode.breakout.BreakoutMotor.Direction.MOTOR_R;

/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
@TeleOp(name = "Mecanum", group = "Pushbot")

public class MecanumDriveOp extends OpMode {

    private static final String NAME = "Mecanum Drive Op";

    //Motor objects
    private Robot robot = new Robot(telemetry);
    private Mecanum drive;

    @Override
    public void init() {
        //Set hardwaremaps for left and right motors
        robot.init(hardwareMap);
        robot.resetAngle();

        //Clear telemetry
        telemetry.clearAll();

        //Mecanum drive handler
        drive = new Mecanum(robot, telemetry);
    }

    @Override
    public void start() {
        //Motor start
        drive.setPower(0, 0, 0);
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
        float[] output;
        if (leftStick1y >= 0.1 || leftStick1y <= -0.1 || rightStick1x >= 0.1 || rightStick1x <= -0.1 || rightTrigger1 != 0 || leftTrigger1 != 0) {
            float turnPower;
            telemetry.addData("sticks moving it", "");
            if (rightTrigger1 != 0 || leftTrigger1 != 0) {
                turnPower = rightTrigger1 - leftTrigger1;
                robot.resetAngle();
            } else {
                turnPower = 0;//(float)robot.checkDirection();
            }
            output = drive.setPower(rightStick1x, leftStick1y, turnPower);
        } else {
            output = drive.setPower(0,0,0);//(float)robot.checkDirection());
            telemetry.addData("moving itself", "");
        }

        //Telemetry
        telemetry.addData("FL", output[0]);
        telemetry.addData("FR", output[1]);
        telemetry.addData("BL", output[2]);
        telemetry.addData("BR", output[3]);
//        telemetry.addData("Left Stick X 1", leftStick1x);
        telemetry.addData("Left Stick Y 1", leftStick1y);
        telemetry.addData("Right Stick X 1", rightStick1x);
//        telemetry.addData("Right Stick Y 1", rightStick1y);
//        telemetry.addData("Left Stick X 2", leftStick2x);
//        telemetry.addData("Left Stick Y 2", leftStick2y);
        RobotLog.i(robot.tag(NAME), "FL " + output[0]);
        RobotLog.i(robot.tag(NAME), "FR " + output[1]);
        RobotLog.i(robot.tag(NAME), "BL " + output[2]);
        RobotLog.i(robot.tag(NAME), "BR " + output[3]);
    }

    @Override
    public void stop() {
        //Motor stop
        drive.setPower(0, 0, 0);
    }
}
