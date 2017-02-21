package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.chassis.Orion;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

import java.io.File;
import java.io.IOException;

/**
 * Created by davis on 10/6/16.
 */
public abstract class AutoV1 extends LinearOpMode {
  Orion robot;

  int SLEEP_TIME = 300;
  int NUM_PUSHES = 1;
  double SPEED = 0.4;
  double FAST_SPEED = 0.8;
  double STRAFE_SPEED = 0.6;
  double ROTATE_SPEED = 0.4;

  String FIRST_TARGET = getFirstTarget();
  String SECOND_TARGET = getSecondTarget();

  abstract String getFirstTarget();
  abstract String getSecondTarget();
  abstract double getDir();
  Settings settings;
  Vuforia vuforia;
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
    vuforia = new Vuforia();
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
      int FORWARD_TICKS = 3000;
      int CAP_BALL_TICKS = 1000;
      driveTicks(-SPEED, FORWARD_TICKS);
      sleep(SLEEP_TIME*2);
      shootParticles();
      sleep(SLEEP_TIME);

      if (settings.knockCapBall) {
        robot.runPickup(1);
        sleep(700);
        robot.runPickup(0);
        driveTicks(-SPEED, CAP_BALL_TICKS);
      }

      stop();
    }

    driveTicks(SPEED, 900);
    sleep(SLEEP_TIME);

    // turn 55 degrees
    rotateDegs(ROTATE_SPEED * getDir(), getDir() == 1 ? 55 : 55);

    // Move forward to get in range of vision targets
    robot.resetTicks();
    double target = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    do {
      long currentTime = System.currentTimeMillis();
      // if we haven't seen the target within 6 seconds, stop moving.
      if (currentTime - startTime > 3000 || Math.abs(robot.getTicks()) > 6400) {
        while (opModeIsActive())
          robot.stopMotors();
      }
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    sleep(SLEEP_TIME);

    // Keep moving until we're 70cm away from the first target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 700 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    }
    robot.stopMotors();
    sleep(SLEEP_TIME);

    // Turn until we're 90 degrees away from how we started.
    rotateDegs(ROTATE_SPEED * getDir(), (getDir() == 1 ? 86 : 75) - 55);
    sleep(SLEEP_TIME);
    // Get current heading for PID strafing
    robot.imu.update();
    double h = robot.imu.heading();
    double dir = getDir();
    telemetry.addData("Dir", dir);
    telemetry.update();
    // Strafe sideways until we're lined up with the vision target
    while ((vuforia.getAlignment(FIRST_TARGET) == null || Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1] + 40) > 50) && opModeIsActive()) {
      robot.imu.update();
      robot.move(STRAFE_SPEED, Math.PI / 2 * dir, 0);
    }
    robot.stopMotors();
    sleep(SLEEP_TIME);

    // Make sure we can see the vision guide before we start moving
    while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive())
      ;

    if (settings.numShots > 0) {
      // Move backwards to align for shooting particle
      while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) < 650 && opModeIsActive()) {
        robot.imu.update();
        robot.moveStraight(2 * SPEED / 3, Math.PI, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
        idle();
      }
      robot.stopMotors();
      robot.centerServo();
      sleep(SLEEP_TIME);

      // Load catapult, and shoot
      shootParticles();
    }

    // move until we're 22.5cm away from the target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 400 && opModeIsActive()) {
      robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
    }
    robot.stopMotors();
    sleep(SLEEP_TIME);
    // Check both sides of beacon, put the right paddle forward.
    int a = robot.hitBeacon(-1 * (int) getDir());

    sleep(SLEEP_TIME);
    // Go forwards and hit the beacon twice to make sure the button is pressed
    int count = 0;
    if (a != 0) {
      while (opModeIsActive() && count < NUM_PUSHES) {
        robot.move(SPEED, 0, 0);
        sleep(700);
        robot.stopMotors();
//        sleep(500);
//        robot.centerServo();
//        sleep(500);
//        if (a == 1) robot.hitLeft();
//        else if (a == -1) robot.hitRight();
        sleep(500);
        robot.move(SPEED, Math.PI, 0);
        sleep(700);
        robot.stopMotors();
        sleep(200);
        count++;
      }
    }
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

    if (settings.beacon2) {
      robot.centerServo();
      robot.imu.resetHeading();
      boolean b = rotateDegs(-ROTATE_SPEED * getDir(), getDir() == 1 ? 90 : 85);
      sleep(SLEEP_TIME);
      driveTicks(FAST_SPEED, 2400);
      sleep(SLEEP_TIME*2);
      robot.imu.resetHeading();
      rotateDegs(ROTATE_SPEED * getDir(), 90);
      sleep(SLEEP_TIME);
      driveTicks(SPEED, b ? 400 : 200);
      sleep(SLEEP_TIME*2);
      while ((vuforia.getAlignment(SECOND_TARGET) == null || Math.abs(Vuforia.getPosition(vuforia.getAlignment(SECOND_TARGET))[1] + (getDir() == 1 ? 40 : 0)) > 50) && opModeIsActive()) {
        robot.imu.update();
        robot.move(STRAFE_SPEED, Math.PI / 2 * dir, 0);
      }
      robot.stopMotors();
      // Make sure we can see the second vision guide
      while (vuforia.getAlignment(SECOND_TARGET) == null && opModeIsActive())
        ;

      sleep(SLEEP_TIME);

      while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(SECOND_TARGET))[2]) > 360 && opModeIsActive()) {
        robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(SECOND_TARGET)));
      }
      robot.stopMotors();
      sleep(SLEEP_TIME);
      // Sense second beacon color, put right paddle forward.
      a = robot.hitBeacon(-1 * (int) getDir());
      sleep(1000);
      count = 0;
      // Hit beacon.
      if (a != 0) {
        while (opModeIsActive() && count < NUM_PUSHES) {
          robot.move(SPEED, 0, 0);
          sleep(700);
          robot.stopMotors();
          sleep(200);
          robot.move(SPEED, Math.PI, 0);
          sleep(700);
          robot.stopMotors();
          sleep(200);
          count++;
        }
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

  /**
   * Move around while orthogonal with a Vuforia vision target.
   *
   * When moving forwards and backwards, `offset` is zero since you're moving until you're a
   * certain distance away.
   *
   * When moving sideways, `threshold` is the margin of error that you're accepting, and `offset`
   * is the phone camera's offset from the center of the robot. When sideways moving, this function
   * basically aligns with the vision guide.
   *
   * @param pow Motor SPEED
   * @param angle direction to move in
   * @param threshold distance in mm away from target to move until
   * @param offset distance
   * @param target string identifier of the vision target to use
   */
  void moveOrthogonalVuforia(double pow, double angle, double threshold, double offset, String target) {
    int i = -1;
    // use distance away for alignment if moving forwards/backwards
    if (angle == Math.PI || angle == 0) i = 2;
    // use position side/to/side relative to vision guide if moving side-to-side
    else if (Math.abs(angle) == Math.PI/2) i = 1;

    robot.imu.update();
    double h = robot.imu.heading();
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(target))[i] - offset) > threshold && opModeIsActive()) {
      robot.imu.heading();
      robot.moveStraight(
              pow, angle,
              Vuforia.getHeading(vuforia.getAlignment(target)));
    }
    robot.stopMotors();
  }

  /**
   * Overload of moveOrthogonalVuforia when no offset is necessary.
   *
   * @param pow motor SPEED
   * @param angle direction to move in
   * @param threshold move until you're `threshold` away
   * @param target vision target to use.
   */
  void moveOrthogonalVuforia(double pow, double angle, double threshold, String target) {
    moveOrthogonalVuforia(pow, angle, threshold, 0, target);
  }

  /**
   *
   * @param pow SPEED
   * @param ticks number of ticks forward
   * @param timeout seconds before you stop moving if encoders don't finish
   */
  void driveTicks(double pow, int ticks, int timeout) {
    double angle = pow > 0 ? 0 : Math.PI;
    robot.resetTicks();
    robot.imu.update();
    double h = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    long currentTime = startTime;
    while (Math.abs(robot.getTicks()) < ticks && currentTime - startTime < timeout && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(Math.abs(pow), angle, robot.imu.heading(), h);
      currentTime = System.currentTimeMillis();
    }
    robot.stopMotors();
  }

  void driveTicks(double pow, int ticks) {
    driveTicks(pow, ticks, 30000);
  }

  boolean rotateDegs(double pow, double degs, int fallbackTicks) {
    double prevHeading = 366;
    boolean b = false;
    robot.resetTicks();
    do {
      robot.imu.update();
      robot.move(0, 0, ROTATE_SPEED * getDir());

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
      sleep(SLEEP_TIME);
    }
    if (settings.numShots > 1) {
      transferParticle();
      sleep(SLEEP_TIME);
      fireParticle();
      sleep(SLEEP_TIME);
    }
  }

  public int pushButton (int color) throws InterruptedException {

    int redLeft, blueLeft;
    robot.colorSensor.enableLed(false);
    redLeft = robot.colorSensor.red();
    blueLeft = robot.colorSensor.blue();
    int hit = 0;

    sleep(250);
    if(redLeft*color > blueLeft*color) {
      robot.pressButton();
      hit = 1;
      sleep(500);
    }

    driveTicks(SPEED/2, 300);
    sleep(500);

    redLeft = robot.colorSensor.red();
    blueLeft = robot.colorSensor.blue();
    sleep(250);

    if(redLeft*color > blueLeft*color && hit==0) {
      robot.pressButton();
      hit = -1;
      sleep(500);
    }
    return hit;
  }
  void alignWithLine() throws InterruptedException{
    while (!robot.isOnLinel()) {
      robot.moveRight(-SPEED);
    }
    robot.stopMotors();
    sleep(250);
    while (!robot.isOnLiner()) {
      robot.moveLeft(-SPEED);
    }
    robot.stopMotors();

  }
}