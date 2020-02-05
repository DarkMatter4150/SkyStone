package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.breakout.EncoderDrive;
import org.firstinspires.ftc.teamcode.breakout.Robot;
import org.firstinspires.ftc.teamcode.field.RobotObject;
import org.firstinspires.ftc.teamcode.mecanum.Mecanum;

/*
 * TODO
 * Arm stick inverted
 * Tab button switched
 * -------------------
 * Stacking heights (elevator)
 * Automate grab block
 *
 */


/**
 * This class is used for the main game to drive the robot using the controllers.
 **/
@TeleOp(name = "Robot Drive", group = "Pushbot")

public class MecanumDriveOpManualServos extends OpMode {

    //Motor objects
    private Robot robot = new Robot(telemetry);
    private RobotObject robotObject = new RobotObject(0, 0, 18, 18, 0);

    private Point savedPoint = new Point();

    private ElapsedTime timer = new ElapsedTime();
    private Mecanum drive;
    private boolean slow = false;
    private boolean claw = false;
    private boolean tabs = true;
    private boolean finger = false;
    private double clawTimer = 0;
    private double tabTimer = 0;
    private double slowTimer = 0;
    private double fingerTimer = 0;

    private double startAngle;
    private double currentRelativeAngle;

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
        startAngle = robot.getAngle();
    }

    @Override
    public void loop() {

        currentRelativeAngle = robot.getAngle() - startAngle;

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
        if (slow) {
                float turnPower;
                if (rightTrigger1 != 0 || leftTrigger1 != 0) {
                    turnPower = rightTrigger1 - leftTrigger1;
                } else {
                    turnPower = 0;
                }
                output = drive.setPower(rightStick1x/2, leftStick1y/2, turnPower/2);
        } else {
                float turnPower;
                if (rightTrigger1 != 0 || leftTrigger1 != 0) {
                    turnPower = rightTrigger1 - leftTrigger1;
                } else {
                    turnPower = 0;
                }
                output = drive.setPower(rightStick1x, leftStick1y, turnPower);
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

        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            robot.setIntakeServos(true);
        } else {
            robot.setIntakeServos(false);
        }

        if (checkColor()) {
            telemetry.addData("SkyStone", true);
        } else {
            telemetry.addData("SkyStone", false);
        }

        if (xButton && timer.milliseconds() - fingerTimer > 250) {
            finger = !finger;
            robot.setFinger(finger);
            fingerTimer = timer.milliseconds();
        }

        if (gamepad1.a) {
            savedPoint.x = EncoderDriverobotObject.getCenterX();

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

    private boolean checkColor() {
        ColorSensor colorSensor = robot.getColorSensor();
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        DistanceSensor distanceSensor = robot.getDistanceSensor();
        float distance = EncoderDrive.toFloat(distanceSensor.getDistance(DistanceUnit.CM));

        return !((red - blue > 30) && (distance < 6.5f) && (red > blue));
    }

    @Override
    public void stop() {
        //Motor stop
        drive.setPower(0, 0, 0);
    }
}
