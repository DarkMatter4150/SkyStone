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
    private ElapsedTime timer = new ElapsedTime();
    private Mecanum drive;
    private boolean slow = false;
    private boolean claw = false;
    private boolean tabs = true;
    private double clawTimer = 0;
    private double tabTimer = 0;
    private double slowTimer = 0;

    @Override
    public void init() {
        //Set hardwaremaps for left and right motors
        robot.init(hardwareMap);

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
        float leftTrigger2 = gamepad2.left_trigger;
        float rightTrigger2 = gamepad2.right_trigger;
        boolean aButton = gamepad2.a;
        boolean bButton = gamepad2.b;
        boolean xButton = gamepad2.x;
        boolean yButton = gamepad2.y;

        //Move the motors//
        float[] output;
//        boolean stickDeadZone = leftStick1y >= 0.1 || leftStick1y <= -0.1 || rightStick1x >= 0.1 || rightStick1x <= -0.1 || rightTrigger1 != 0 || leftTrigger1 != 0;
        if (slow) {
//            if (stickDeadZone) {
                float turnPower;
                if (rightTrigger1 != 0 || leftTrigger1 != 0) {
                    turnPower = rightTrigger1 - leftTrigger1;
                } else {
                    turnPower = 0;
                }
                output = drive.setPower(rightStick1x/2, leftStick1y/2, -turnPower/2);
//            } else {
//                output = drive.setPower(0, 0, 0);
//            }
        } else {
//            if (stickDeadZone) {
                float turnPower;
                if (rightTrigger1 != 0 || leftTrigger1 != 0) {
                    turnPower = rightTrigger1 - leftTrigger1;
                } else {
                    turnPower = 0;
                }
                output = drive.setPower(rightStick1x, leftStick1y, turnPower);
//            } else {
//                output = drive.setPower(0, 0, 0);
//            }
        }

        if (aButton && timer.milliseconds() - tabTimer > 250) {
            tabs = !tabs;
            robot.setTabs(tabs);
            tabTimer = timer.milliseconds();
        }

        if (yButton && timer.milliseconds() - clawTimer > 250) {
            claw = !claw;
            robot.setClaw(claw);
            clawTimer = timer.milliseconds();
        }

        if (gamepad1.a && timer.milliseconds() - slowTimer > 250) {
            slow = !slow;
            slowTimer = timer.milliseconds();
        }

        robot.setWheelIntake(leftTrigger2-rightTrigger2);

        //Arm
        robot.moveArm(leftStick2y);

        //Telemetry
        telemetry.addData("FL", output[0]);
        telemetry.addData("FR", output[1]);
        telemetry.addData("BL", output[2]);
        telemetry.addData("BR", output[3]);
        telemetry.addData("FL Power Float", robot.getPower(Robot.Motor.FRONT_LEFT));
        telemetry.addData("FR Power Float", robot.getPower(Robot.Motor.FRONT_RIGHT));
        telemetry.addData("BL Power Float", robot.getPower(Robot.Motor.BACK_LEFT));
        telemetry.addData("BR Power Float", robot.getPower(Robot.Motor.BACK_RIGHT));
        telemetry.addData("Claw Pos", robot.getClawPos());
        telemetry.addData("slow?", slow);
    }

    @Override
    public void stop() {
        //Motor stop
        drive.setPower(0, 0, 0);
    }
}
