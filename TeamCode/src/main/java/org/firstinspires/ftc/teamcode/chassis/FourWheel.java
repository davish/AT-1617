package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.sensors.Gyro;

/**
 * Created by davis on 9/27/16.
 */
public abstract class FourWheel {
  HardwareMap hwMap;
  DcMotor FL; // v1
  DcMotor FR; // v2
  DcMotor BL; // v3
  DcMotor BR; // v4

  DcMotor choo;
  DcMotor pickup;

  public ColorSensor colorSensor;
  public AnalogInput lineSensor;
  public Gyro imu;
  public OpticalDistanceSensor ods;
  DigitalChannel chooLimit;

  Servo beacon;

  DigitalChannel blueLights;
  DigitalChannel redLights;
  DigitalChannel greenLights;

  public final double PIVOT_SENSELEFT = .45;
  public final double PIVOT_CENTER = .55;
  public final double PIVOT_SENSERIGHT = .65;
  public final double PIVOT_HITLEFT = .73;
  public final double PIVOT_HITRIGHT = .38;

  public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;
    FL = hwMap.dcMotor.get("FL");
    FR = hwMap.dcMotor.get("FR");
    BL = hwMap.dcMotor.get("BL");
    BR = hwMap.dcMotor.get("BR");


    FL.setDirection(DcMotor.Direction.REVERSE);
    BL.setDirection(DcMotor.Direction.REVERSE);

    choo = hwMap.dcMotor.get("choo");
    choo.setDirection(DcMotor.Direction.REVERSE);

    pickup = hwMap.dcMotor.get("pickup");

    beacon = hwMap.servo.get("beacon");
    beacon.setPosition(PIVOT_CENTER);

    chooLimit = hwMap.digitalChannel.get("choo limit");
    chooLimit.setMode(DigitalChannelController.Mode.INPUT);

    colorSensor = hwMap.colorSensor.get("mr");
    imu = new Gyro(hwMap.get(BNO055IMU.class, "imu"));
    ods = hwMap.opticalDistanceSensor.get("ods");
  }

  public boolean isOnLine() {
    return ods.getLightDetected() > .5;
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

  public void runPickup(double pow) {
    pickup.setPower(pow);
  }

  public boolean catapultLoaded() {
    return !chooLimit.getState();
  }

  public void runChoo(double pow) {
    choo.setPower(pow);
  }

  public int hitBeacon(int color) throws InterruptedException {
    int redLeft, blueLeft, redRight, blueRight;
    this.colorSensor.enableLed(false);
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

    if (redLeft*color > redRight*color) {
      this.hitLeft();
      return 1;
    }
    else if (redRight*color > redLeft*color) {
      this.hitRight();
      return -1;
    }
    else if (blueLeft*color > blueRight*color) {
      this.hitRight();
      return -1;
    }
    else if (blueRight*color > blueLeft*color) {
      this.hitLeft();
      return 1;
    }
    else
      return 0;

  }


}
