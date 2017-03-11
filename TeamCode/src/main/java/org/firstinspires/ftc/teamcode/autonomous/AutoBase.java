package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Atlas;

import java.io.File;
import java.io.IOException;
import java.io.InterruptedIOException;

/**
 * Created by davis on 11/25/16.
 */
public abstract class AutoBase extends LinearOpMode {
  Atlas robot;
  Settings settings;

  int SLEEP_TIME = 300;
  int NUM_PUSHES = 1;
  double SPEED = -0.4; // -.3 definitely works, trying faster speed.
  double FAST_SPEED = -0.8;
  double STRAFE_SPEED = 0.6;
  double ROTATE_SPEED = 0.5;



  static final double FORWARD = 0;
  static final double BACKWARD = Math.PI;


  void setup() throws InterruptedException{
    Gson gson = new Gson();
    File sfile = AppUtil.getInstance().getSettingsFile("auto_settings.json");
    try {
      settings = gson.fromJson(ReadWriteFile.readFileOrThrow(sfile), Settings.class);
    } catch (IOException e) {
      settings = new Settings();
    }
    robot = new Atlas();
    robot.init(hardwareMap);
    while (!robot.catapultLoaded())
      robot.runChoo(1);
    robot.runChoo(0);
    telemetry.addData("Delay (seconds)", settings.delay);
    telemetry.addData("Hit beacon 1", settings.beacon1);
    telemetry.addData("Hit beacon 2", settings.beacon2);
    telemetry.addData("Shoot how many particles", settings.numShots);
    telemetry.addData("Knock off cap ball", settings.knockCapBall);
    telemetry.addData("End on center", settings.endOnCenter);
    telemetry.update();
    telemetry.update();
  }

  abstract void run() throws InterruptedException;

  public void runOpMode() throws InterruptedException{
    setup();
    waitForStart();
    telemetry.addData(">", settings.delay + " second delay");
    telemetry.update();
    sleep(settings.delay*1000);
    telemetry.update();
    run();
  }

  abstract double getDir();


  void transferParticle() throws InterruptedException{
    double chamberPos = robot.REST_POSITION;
    while (opModeIsActive() && chamberPos > robot.LOAD_POSITION) {
      chamberPos -= robot.STEP_SIZE;
      robot.transervo(chamberPos);
      idle();
    }
    sleep(robot.DELAY_TIME);
    while (opModeIsActive() && chamberPos < robot.REST_POSITION) {
      chamberPos += robot.STEP_SIZE;
      robot.transervo(chamberPos);
      idle();
    }
  }
  void fireParticle() throws InterruptedException {
    while (!robot.catapultLoaded() && opModeIsActive())
      robot.runChoo(1);
    while (robot.catapultLoaded() && opModeIsActive())
      robot.runChoo(1);
    while (!robot.catapultLoaded() && opModeIsActive())
      robot.runChoo(1);
    robot.runChoo(0);
  }

  void moveTicks(double pow, double angle, int ticks, int timeout) {
    robot.resetTicks();
    robot.imu.update();
    double h = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    long currentTime = startTime;
    while (Math.abs(robot.getTicks()) < ticks && currentTime - startTime < timeout && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(Math.abs(pow), angle, robot.imu.heading(), h);
      currentTime = System.currentTimeMillis();
      telemetry.addData("Target", ticks);
      telemetry.addData("Current", robot.getTicks());
      telemetry.update();
    }
    robot.stopMotors();
  }

  /**
   *
   * @param pow SPEED
   * @param ticks number of ticks forward
   * @param timeout seconds before you stop moving if encoders don't finish
   */
  void driveTicks(double pow, int ticks, int timeout) {
    double angle = pow > 0 ? 0 : Math.PI;
    moveTicks(pow, angle, ticks, timeout);
  }

  void moveTicks(double pow, double angle, int ticks) {
    moveTicks(pow, angle, ticks, 30000);
  }

  void driveTicks(double pow, int ticks) {
    driveTicks(pow, ticks, 30000);
  }

  boolean rotateDegs(double pow, double degs, int fallbackTicks) {
    double prevHeading = 366;
    boolean b = false;
    robot.resetTicks();
    robot.imu.resetHeading();
    do {
      robot.imu.update();
      robot.move(0, 0, pow);

//      telemetry.addData("degrees", robot.imu.heading());
//      telemetry.update();
//      if (prevHeading == robot.imu.heading()) {
//        b = true;
//        break;
//      }
//      prevHeading = robot.imu.heading();
    } while (Math.abs(robot.imu.heading()) < degs && opModeIsActive());

//    while (b && Math.abs(robot.getTicks()) < fallbackTicks && opModeIsActive()) {
//      robot.move(0, 0, pow);
//      telemetry.addData("ticks", robot.getTicks());
//      telemetry.update();
//    }

    robot.stopMotors();
    return !b;
  }
  boolean rotateDegs(double pow, double degs) {
    return rotateDegs(pow, degs, 0);
  }

  void shootParticles() throws InterruptedException {
    if (settings.numShots > 0) {
      fireParticle();
//      sleep(SLEEP_TIME);
    }
    if (settings.numShots > 1) {
      transferParticle();
      sleep(SLEEP_TIME);
      fireParticle();

    }
  }

  void alignWithWall() throws InterruptedException {
    double frontDist;
    double backDist;
    do {
      frontDist = robot.getFrontDistance();
      backDist = robot.getBackDistance();
      robot.move(0, 0, ROTATE_SPEED * FtcUtil.sign(frontDist - backDist));
      telemetry.addData("front", frontDist);
      telemetry.addData("back", backDist);
      telemetry.update();
    } while (Math.abs(frontDist - backDist) > 1.0 && opModeIsActive());
    robot.stopMotors();
  }

  void alignWithLine() throws InterruptedException {
    double dir = -1;
//    while (!(robot.isOnLinel() && robot.isOnLiner()) && opModeIsActive()) {
//
//    }

    while (!robot.isOnLinel() && opModeIsActive()) {
      robot.drive(SPEED*dir, SPEED*dir, 0, 0);
    }
    while  (!robot.isOnLiner() && opModeIsActive()) {
      robot.drive(0, 0, SPEED*dir, SPEED*dir);
    }

    robot.stopMotors();
  }

  public int pushButton (int color, boolean second) throws InterruptedException {
    int redLeft, blueLeft;
    robot.colorSensor.enableLed(false);
    redLeft = robot.colorSensor.red();
    blueLeft = robot.colorSensor.blue();
    print("Red:" + redLeft + ", Blue: " + blueLeft);
//    redLeft = 1;
//    blueLeft = 0;
    int hit = 0;
    if(redLeft*color > blueLeft*color) {
      if (second) pushSecond();
      else pushFirst();
      hit = 1;
      sleep(500);
    }

    if (hit==0) {
      driveTicks(SPEED / 2 * getDir(), 290);
      sleep(500);

      redLeft = robot.colorSensor.red();
      blueLeft = robot.colorSensor.blue();
      print("Red:" + redLeft + ", Blue: " + blueLeft);

      if (redLeft * color > blueLeft * color) {
        if (second) pushSecond();
        else pushFirst();
        hit = -1;
      }
    }
    return hit;
  }

  void pushFirst() {
    robot.pushOut();
    sleep(2000);
    robot.pushStop();
//    sleep(200);
    moveTicks(STRAFE_SPEED, -Math.PI / 2, 500, 500);
    sleep(200);
    robot.pushIn();
    moveTicks(STRAFE_SPEED, Math.PI / 2, 700, 1000);
    sleep(1500);
    robot.pushStop();
  }

  void pushSecond() {
    robot.pushOut();
    sleep(2000);
    robot.pushStop();
//    sleep(200);
    moveTicks(.8, -Math.PI / 2, 500, 500);
    sleep(200);
    robot.pushIn();
    moveTicks(.8, Math.PI / 2, 250, 500);
    sleep(1500);
    robot.pushStop();
//    moveTicks(STRAFE_SPEED, Math.PI / 2, 500, 500);
  }

  void approachBeacon() {
    if (getDir() > 0) {
      print("going forward...");
      driveTicks(SPEED * getDir(), 300);
      robot.stopMotors();
      sleep(SLEEP_TIME);
      print("finding line...");
      moveUntilOnLine(SPEED / 2, BACKWARD);
      print("line found.");
      sleep(SLEEP_TIME);
      // drive forward to align with beacon, then push the proper button
      print("drive forwards");
      driveTicks(SPEED / 2 * getDir(), 200);
    } else if (getDir() < 0) {
      moveUntilOnLine(SPEED / 2, BACKWARD);
      sleep(SLEEP_TIME);
      driveTicks(SPEED, 400);
    }
  }

  /**
   * Move until range sensor reads less than the given distance
   * @param dist distance threshold in centimeters
   * @param pow SPEED to move with
   * @param angle angle to move at
   * @throws InterruptedException
     */
  void moveUntilCloserThan(double dist, double pow, double angle) {
    robot.imu.update();
    double h = robot.imu.heading();
    while (robot.getDistance() > dist && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(pow, angle, robot.imu.heading(), h);
    }
    robot.stopMotors();
  }

  /**
   * Move in direction that range senor can see (right, with respect to nom) until range sensor
   * reads less than the given distance
   * @param dist distance threshold in centimeters
   * @param pow approximate motor SPEED
     */
  void moveUntilCloserThan(double dist, double pow) {
    moveUntilCloserThan(dist, pow, -Math.PI/2);
  }

  /**
   * Move robot until it senses a white line.
   * @param pow speed to move at
   * @param angle angle to move at.
     */
  void moveUntilOnLine(double pow, double angle) {
    while (!robot.isOnLinel() && opModeIsActive()) {
      robot.move(pow, angle, 0);
    }
    robot.stopMotors();
  }
  void print(String s) {
    telemetry.addData(">", s);
    telemetry.update();
  }
}
