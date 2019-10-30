package org.firstinspires.ftc.teamcode.mecanum;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.breakout.Robot;

public class Mecanum {

    private Robot robot;
    private Telemetry telemetry;

    public Mecanum(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public float[] setPower(float x, float y, float z) {
        float FL = y + z + x;
        float FR = y - z - x;
        float BL = y + z - x;
        float BR = y - z + x;
        float[] output = normalize(FL, FR, BL, BR);
        robot.setPower(Robot.Motor.FRONT_LEFT, output[0]);
        robot.setPower(Robot.Motor.FRONT_RIGHT, output[1]);
        robot.setPower(Robot.Motor.BACK_LEFT, output[2]);
        robot.setPower(Robot.Motor.BACK_RIGHT, output[3]);
        telemetry.addData("FL in mecanum class", output[0]);
        telemetry.addData("FR in mecanum class", output[1]);
        telemetry.addData("BL in mecanum class", output[2]);
        telemetry.addData("BR in mecanum class", output[3]);
        return output;
    }

    private float[] normalize(float fl, float fr, float bl, float br) {
        float t0 = Math.max(Math.abs(fl), Math.abs(fr));
        float t1 = Math.max(Math.abs(bl), Math.abs(br));
        float max = Math.max(t0, t1);

        float frontLeft;
        float frontRight;
        float backLeft;
        float backRight;

        if (max < 1) {
            frontLeft = fl;
            frontRight = fr;
            backLeft = bl;
            backRight = br;
        } else {
            frontLeft = fl/max;
            frontRight = fr/max;
            backLeft = bl/max;
            backRight = br/max;
        }

        if (frontLeft < 0.2 && frontLeft > -0.2) {
            frontLeft = 0;
        }
        if (frontRight < 0.2 && frontRight > -0.2) {
            frontRight = 0;
        }
        if (backLeft < 0.2 && backLeft > -0.2) {
            backLeft = 0;
        }
        if (backRight < 0.2 && backRight > -0.2) {
            backRight = 0;
        }

        return new float[]{frontLeft, frontRight, backLeft, backRight};
    }

    @Deprecated
    public void setRotationPower(float left, float right) {
        float leftside = right-left;
        float rightside = left-right;

        robot.setPower(Robot.Motor.FRONT_LEFT, leftside);
        robot.setPower(Robot.Motor.FRONT_RIGHT, rightside);
        robot.setPower(Robot.Motor.BACK_LEFT, leftside);
        robot.setPower(Robot.Motor.BACK_RIGHT, rightside);
    }
}
