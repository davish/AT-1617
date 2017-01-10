package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

/**
 * Created by davis on 12/6/16.
 */
public class Orion {
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
  public OpticalDistanceSensor odsl;
  public OpticalDistanceSensor odsr;

  DigitalChannel chooLimit;

  public Servo beacon;

  public DigitalChannel blueLights;
  public DigitalChannel redLights;
  public DigitalChannel greenLights;

  public AnalogInput distl;
  public AnalogInput distr;

  public final double PIVOT_SENSELEFT = .45;
  public final double PIVOT_CENTER = .55;
  public final double PIVOT_SENSERIGHT = .65;
  public final double PIVOT_HITLEFT = .73;
  public final double PIVOT_HITRIGHT = .38;
  public final double PIVOT_LOADBALL = .53;


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
//    ods = hwMap.opticalDistanceSensor.get("ods");
//    dist=  hwMap.analogInput.get("distance");
//    odsl = hwMap.opticalDistanceSensor.get("odsl");
//    odsr = hwMap.opticalDistanceSensor.get("odsr");
//    distl =  hwMap.analogInput.get("distancel");
//    distr =  hwMap.analogInput.get("distancer");


    // Lights
//    blueLights = hwMap.digitalChannel.get("blue");
//    blueLights.setMode(DigitalChannelController.Mode.OUTPUT);
//    redLights = hwMap.digitalChannel.get("red");
//    redLights.setMode(DigitalChannelController.Mode.OUTPUT);
//    greenLights = hwMap.digitalChannel.get("green");
//    greenLights.setMode(DigitalChannelController.Mode.OUTPUT);
    resetTicks();
  }

  public boolean isOnLinel() {
    return odsl.getLightDetected() > .5;
  }
  public boolean isOnLiner() {
    return odsr.getLightDetected() > .5;
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
  public void loadingPos() {
    pivot(PIVOT_LOADBALL);
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

  public void lineFollow() throws InterruptedException
  {
    if(!this.isOnLinel())
    {
      this.moveLeft(.5);
    }
    if(!this.isOnLiner())
    {
      this.moveRight(.5);
    }
    else{
      this.driveTank(.5, .5);
    }
  }

  public int hitBeacon(int color) throws InterruptedException {
    // Look at both sides of the color sensor, and from that information, decide which side is
    // "bluer" and which side is "redder", and determine which side to hit based off that.
    // Red is 1, blue is -1. the "color" int flips the comparisons which changes which
    // color gets hit.
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

    if (redLeft*color > redRight*color) { // if one side is "redder", use that.
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

  public boolean seesRed() {
    return this.colorSensor.red() > this.colorSensor.blue();
  }
  public boolean seesBlue() {
    return this.colorSensor.blue() > this.colorSensor.red();
  }

    double integral = 0.0; // Accumulation of error over time. Used in PID controller.
  double lastError = 0.0;

  /**
   * Use PID in order to move directly in the desired angle, without rotation.
   * @param pow Motor power (approximate speed)
   * @param angle Desired angle
   * @param actual Orientation/rotation measurement
   * @param target Desired orientation/rotation
   */
  public void moveStraight(double pow, double angle, double actual, double target) {
    double error = actual - target;
    integral += error;

    double change = (error - lastError);
    lastError = error;

    double PID = getKp()*error + getKi()*integral + getKd()*change;

    double rot = FtcUtil.motorScale(PID);
    this.move(pow, angle, rot);
  }

  /**
   * Use PID to move without rotation, when the target angle is 0.
   */
  public void moveStraight(double pow, double angle, double actual) {
    this.moveStraight(pow, angle, actual, 0);
  }

  public void turnDegs(double pow, double degs) {
    this.imu.resetHeading();
    while (Math.abs(this.imu.heading()) < degs) {
      this.move(0, 0, pow);
    }
    this.stopMotors();
  }

  public void alignWithTarget(float[] pos, float heading, double SPEED) {
    double pow, angle;

    if (Math.abs(pos[1] - getPhoneOffset()) > 50) {
      pow = SPEED;
      angle = FtcUtil.sign(pos[1]) * -Math.PI/2;
    }
    else {
      angle = 0;
      pow = SPEED;
    }

    this.moveStraight(pow, angle, heading);
  }

  double getKp() {
    return -0.02;
  }
  double getKi() {
    return 0.0;
  }
  double getKd() {
    return 0.0;
  }

  /**
   * Drive in a certain direction with a mecanum chassis
   * @param pow Base power (magnitude)
   * @param angle Angle to drive towards
   * @param rot speed of rotation
   */
  public void move(double pow, double angle, double rot) {
    pow = FtcUtil.motorScale(pow);
    rot = FtcUtil.motorScale(rot);


    // Adding PI/4 ensures that 0 degrees is straight ahead
    double vx = pow*Math.cos(angle+Math.PI/4);
    double vy = pow*Math.sin(angle+Math.PI/4);

    double[] V = {vx+rot, vy-rot, vy+rot, vx-rot}; // contains motor powers for each motor.

    /*
     * because of adding/subtracting rotation, these numbers could be between [-2,2].
     * To get around this, find the maximum motor power, and divide all of them by that
     * so that the proportions stay the same but now it's between [-1,1].
     */

    // find max
    double m = 0.0;
    for (double v : V)
      if (Math.abs(v) > m)
        m = v;

    double mult = Math.max(Math.abs(pow), Math.abs(rot)); // If we're just rotating, pow will be 0
    // adjust values, still keeping power in mind.
    if (m != 0) // if the max power isn't 0 (can't divide by 0)
      for(int i = 0; i < V.length; i++)
        V[i] = Math.abs(mult) * (V[i]/Math.abs(m));

    // finally, set motor powers.
    FL.setPower(FtcUtil.motorScale(V[0]));
    FR.setPower(FtcUtil.motorScale(V[1]));
    BL.setPower(FtcUtil.motorScale(V[2]));
    BR.setPower(FtcUtil.motorScale(V[3]));

  }

  int pos = 0;
  public int getTicks() {
    return BL.getCurrentPosition() - pos;
  }
  public void resetTicks() {
    pos = BL.getCurrentPosition();
  }

  public double getPhoneOffset() {
    return 120;
  }
}
