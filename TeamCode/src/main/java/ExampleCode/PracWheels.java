package ExampleCode;//package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Wheels class.
 * Enables control of the robot movement via the game_pad.
 */
public class PracWheels {




  private DcMotor motorLeft;
  private DcMotor motorRight;

  private void setMotorMode(DcMotor.RunMode mode) {
    motorLeft.setMode(mode);
    motorRight.setMode(mode);
  }

  private void setPower(double left, double right) {
    motorLeft.setPower(left);
    motorRight.setPower(right);
  }

  public PracWheels() { }

  public void init(DcMotor left, DcMotor right) {
    motorLeft  = left;
    motorRight = right;

    motorLeft.setDirection(DcMotor.Direction.FORWARD);
      motorRight.setDirection(DcMotor.Direction.REVERSE);

    setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void start() { setPower(0.0, 0.0); }

  public void move(double left, double right)  {
    setPower(left, right);
  }



  public void stop() { setPower(0.0, 0.0); }

} // Wheels
