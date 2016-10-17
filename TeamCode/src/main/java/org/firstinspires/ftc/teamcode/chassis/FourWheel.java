package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by davis on 9/27/16.
 */
public abstract class FourWheel {
  HardwareMap hwMap;
  DcMotor FL; // v1
  DcMotor FR; // v2
  DcMotor BL; // v3
  DcMotor BR; // v4

  public ColorSensor colorSensor;
  Servo beacon;

  public final double PIVOT_SENSELEFT = .65;
  public final double PIVOT_CENTER = .75;
  public final double PIVOT_SENSERIGHT = .85;
  public final double PIVOT_HITLEFT = 1.0;
  public final double PIVOT_HITRIGHT = .45;

  public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;
    FL = hwMap.dcMotor.get("FL");
    FR = hwMap.dcMotor.get("FR");
    BL = hwMap.dcMotor.get("BL");
    BR = hwMap.dcMotor.get("BR");

    FL.setDirection(DcMotor.Direction.REVERSE);
    BL.setDirection(DcMotor.Direction.REVERSE);

    colorSensor = hwMap.colorSensor.get("mr");
    beacon = hwMap.servo.get("beacon");
    beacon.setPosition(PIVOT_CENTER);
  }

  public void moveLeft(double pow) {
    FL.setPower(pow);
    BL.setPower(pow);
  }

  public void moveRight(double pow) {
    FR.setPower(pow);
    BR.setPower(pow);
  }

  public void driveTank(double l, double r) {
    moveLeft(l);
    moveRight(r);
  }

  public void stopMotors() {
    driveTank(0, 0);
  }

  public void pivot(double pos) {
    beacon.setPosition(pos);
  }

  public void senseLeft() {
    pivot(PIVOT_SENSELEFT);
  }
  public void senseRight() {
    pivot(PIVOT_SENSERIGHT);
  }
  public void hitLeft() {
    pivot(PIVOT_HITLEFT);
  }
  public void hitRight() {
    pivot(PIVOT_HITRIGHT);
  }
  public void centerServo() {
    pivot(PIVOT_CENTER);
  }

  public void hitBeacon(int color) throws InterruptedException {
    int redLeft, blueLeft, redRight, blueRight;
    this.senseLeft();
    Thread.sleep(500);
    redLeft = this.colorSensor.red();
    blueLeft = this.colorSensor.blue();
    this.senseRight();
    Thread.sleep(500);
    redRight = this.colorSensor.red();
    blueRight = this.colorSensor.blue();
    this.centerServo();
    Thread.sleep(500);

    if (redLeft*color > redRight*color)
      this.hitLeft();
    else if (redRight*color > redLeft*color)
      this.hitRight();
    else if (blueLeft*color > blueRight*color)
      this.hitRight();
    else if (blueRight*color > blueLeft*color)
      this.hitLeft();
  }
}
