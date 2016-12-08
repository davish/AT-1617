package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Orion;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

import java.io.File;
import java.io.IOException;

/**
 * Created by davis on 10/6/16.
 */
public abstract class AutoV1 extends LinearOpMode {
  Orion robot;

  double SPEED = 0.4;
  double FAST_SPEED = 0.6;
  double ROTATE_SPEED = 0.4;

  String FIRST_TARGET = getFirstTarget();
  String SECOND_TARGET = getSecondTarget();

  abstract String getFirstTarget();
  abstract String getSecondTarget();
  abstract double getDir();
  Settings settings;
  public void runOpMode() throws InterruptedException {

    Gson gson = new Gson();
    File sfile = AppUtil.getInstance().getSettingsFile("auto_settings.json");
    try {
      settings = gson.fromJson(ReadWriteFile.readFileOrThrow(sfile), Settings.class);
    } catch (IOException e) {
      settings = new Settings();
    }
    robot = new Orion();
    robot.init(hardwareMap);
    Vuforia vuforia = new Vuforia();
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

    waitForStart();
    vuforia.activate();
    sleep(settings.delay * 1000);

    if (!settings.beacon1 && !settings.beacon2 && settings.numShots > 0) {
      int FORWARD_TICKS = 900;
      int CAP_BALL_TICKS = 1200;

      driveTicks(SPEED, FORWARD_TICKS);
      sleep(500);
      shootParticles();
      sleep(500);

      if (settings.knockCapBall) {
        driveTicks(SPEED, CAP_BALL_TICKS);
      }

      stop();
    }

    robot.resetTicks();
    while (Math.abs(robot.getTicks()) < 900 && opModeIsActive())
      robot.move(SPEED, 0, 0);
    robot.stopMotors();
    sleep(500);

    // turn 55 degrees
    do {
      robot.imu.update();
      robot.move(0, 0, ROTATE_SPEED * getDir());
    } while (Math.abs(robot.imu.heading()) < (getDir() == 1 ? 55 : 60) && opModeIsActive());
    robot.stopMotors();
    sleep(500);

    // Move forward to get in range of vision targets
    robot.resetTicks();
    double target = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    do {
      long currentTime = System.currentTimeMillis();
      // if we haven't seen the target within 6 seconds, stop moving.
      if (currentTime - startTime > 3000 || Math.abs(robot.getTicks()) > 6700) {
        while (opModeIsActive())
          robot.stopMotors();
      }
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    sleep(500);
    // Keep moving until we're 70cm away from the first target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 700 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    }
    robot.stopMotors();
    sleep(500);

    // Turn until we're 90 degrees away from how we started.
    do {
      robot.imu.update();
      robot.move(0, 0, ROTATE_SPEED * getDir());
    } while (Math.abs(robot.imu.heading()) < (getDir() == 1 ? 86 : 75) && opModeIsActive());
    robot.stopMotors();

    sleep(500);
    // Get current heading for PID strafing
    robot.imu.update();
    double h = robot.imu.heading();

    // get our position relative to the vision target (are we on left/right side)
    double dir = FtcUtil.sign(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]);
    telemetry.addData("Dir", dir);
    telemetry.update();
    // Strafe sideways until we're lined up with the vision target
    while (vuforia.getAlignment(FIRST_TARGET) == null || Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1] + 40) > 50) {
      robot.imu.update();
      robot.move(FAST_SPEED, Math.PI / 2 * dir, 0);
    }
    robot.stopMotors();
    sleep(500);

    // Make sure we can see the vision guide before we start moving
    while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive())
      ;
    // Move backwards to align for shooting particle
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) < 650 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, Math.PI, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
      idle();
    }
    robot.stopMotors();
    robot.centerServo();
    sleep(500);

//    // Load catapult, and shoot
    shootParticles();

    // move until we're 22.5cm away from the target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 360 && opModeIsActive()) {
      robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    // Check both sides of beacon, put the right paddle forward.
    int a = robot.hitBeacon(-1 * (int) getDir());

    sleep(1000);
    // Go forwards and hit the beacon twice to make sure the button is pressed
    int count = 0;
    if (a != 0) {
      while (opModeIsActive() && count < 2) {
        robot.move(2 * SPEED / 3, 0, 0);
        sleep(700);
        robot.stopMotors();
        sleep(250);
        robot.move(2 * SPEED / 3, Math.PI, 0);
        sleep(700);
        robot.stopMotors();
        sleep(250);
        count++;
      }
    }
    sleep(500);
    robot.imu.update();
    h = robot.imu.heading();

    // 2400 ticks between beacons

    if (settings.knockCapBall) {
      robot.resetTicks();
      while (Math.abs(robot.getTicks()) < 1700 && opModeIsActive()) {
        robot.move(SPEED, Math.PI, 0);
      }
      robot.stopMotors();
    }

    if (!settings.beacon2)
      while (opModeIsActive())
        robot.stopMotors();

    robot.imu.resetHeading();
    do {
      robot.imu.update();
      robot.move(0, 0, -ROTATE_SPEED * getDir());
    } while (Math.abs(robot.imu.heading()) < 90 && opModeIsActive());
    robot.stopMotors();
    while (opModeIsActive())
      robot.stopMotors();

    while (opModeIsActive() && (vuforia.getAlignment(SECOND_TARGET) == null || Math.abs(Vuforia.getPosition(vuforia.getAlignment(SECOND_TARGET))[1] - (getDir() == 1 ? 40 : -20)) > 50)) {
      robot.imu.update();
      robot.move(FAST_SPEED, Math.PI / 2 * dir, 0);
    }

    // Make sure we can see the second vision guide
    while (vuforia.getAlignment(SECOND_TARGET) == null && opModeIsActive())
      ;

    // Align with vision target and move until we're 22.5cm away
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(SECOND_TARGET))[2]) > 225 && opModeIsActive()) {
      robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(SECOND_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    // Sense second beacon color, put right paddle forward.
    a = robot.hitBeacon(-1 * (int) getDir());
    sleep(1000);
    count = 0;
    // Hit beacon.
    if (a != 0) {
      while (opModeIsActive() && count < 2) {
        robot.move(1 * SPEED / 3, 0, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        robot.move(1 * SPEED / 3, Math.PI, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        count++;
      }
    }
  }

  void transferParticle() throws InterruptedException{
    double chamberPos = robot.PIVOT_LOADBALL;
    while (opModeIsActive() && chamberPos > robot.PIVOT_HITRIGHT) {
      chamberPos -= .01;
      robot.pivot(chamberPos);
      idle();
    }
    sleep(150);
    while (opModeIsActive() && chamberPos < robot.PIVOT_LOADBALL) {
      chamberPos += .01;
      robot.pivot(chamberPos);
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

  void driveTicks(double pow, int ticks, int timeout) {
    robot.resetTicks();
    robot.imu.update();
    double h = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    long currentTime = startTime;
    while (Math.abs(robot.getTicks()) < ticks && currentTime - startTime < timeout && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(pow, 0, robot.imu.heading(), h);
      currentTime = System.currentTimeMillis();
    }
    robot.stopMotors();
  }

  void driveTicks(double pow, int ticks) {
    driveTicks(pow, ticks, 30000);
  }

  void rotateDegs(double pow, double degs) {
    do {
      robot.imu.update();
      robot.move(0, 0, pow);
    } while (Math.abs(robot.imu.heading()) < degs && opModeIsActive());
    robot.stopMotors();
  }

  void shootParticles() throws InterruptedException {
    if (settings.numShots > 0) {
      fireParticle();
      sleep(500);
    }
    if (settings.numShots > 1) {
      transferParticle();
      sleep(500);
      fireParticle();
      sleep(500);
    }
  }
}