package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Atlas;

import java.io.File;
import java.io.IOException;

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


  /**
   * Initialize the robot, load settings, and display current settings on driver station.
   * @throws InterruptedException
   */
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
    while (!robot.catapultLoaded()) // load catapult as part of initialization.
      robot.runChoo(1);
    robot.runChoo(0);
    telemetry.addData("Delay (seconds)", settings.delay);
    telemetry.addData("Hit beacon 1", settings.beacon1);
    telemetry.addData("Hit beacon 2", settings.beacon2);
    telemetry.addData("Shoot how many particles", settings.numShots);
    telemetry.addData("Knock off cap ball", settings.knockCapBall);
    telemetry.addData("Sprint to Ramp", settings.endOnCenter);
    telemetry.update();
    telemetry.update();
  }

  /**
   * Code for specific autonomous program goes here.
   * @throws InterruptedException
   */
  abstract void run() throws InterruptedException;

  public void runOpMode() throws InterruptedException{
    setup();
    waitForStart();
    telemetry.addData(">", settings.delay + " second delay");
    telemetry.update();
    sleep(settings.delay*1000); // delay running OpMode as much as the settings app tells you to.
    telemetry.update();
    run();
  }


  long startPaddleAction = -1;
  long paddleActionDuration = -1;
  /**
   * This function should be called at the end of every loop in this program.
   * It manages actions that are being done asynchronously.
   */
  void manageAsync() {
    // if the button pusher is moving it has exceeded its duration, stop the paddle.
    if (startPaddleAction != -1 && System.currentTimeMillis() - startPaddleAction > paddleActionDuration) {
      robot.pushStop();
      startPaddleAction = paddleActionDuration = -1;
    }
  }

  void asyncPushOut(int millis) {
    startPaddleAction = System.currentTimeMillis();
    paddleActionDuration = millis;
    robot.pushOut();
  }

  void asyncPushIn(int millis) {
    startPaddleAction = System.currentTimeMillis();
    paddleActionDuration = millis;
    robot.pushIn();
  }

  /**
   * Get the alliance for the current OpMode. useful for changing direction of turns etc.
   * depending on which side of the field we're on.
   * @return 1 for red, -1 for blue.
   */
  abstract double getDir();


  /**
   * Transfer a particle from the transfer ramp into the catapult.
   * Takes advantage of servo positions that are set in the Atlas class.
   * To change speed of transfer for both Autonomous and TeleOp, change values in
   * Atlas.
   *
   * @throws InterruptedException
   */
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

  /**
   * Shoot a particle.
   * @throws InterruptedException
   */
  void fireParticle() throws InterruptedException {
    while (!robot.catapultLoaded() && opModeIsActive()) // load the catapult
      robot.runChoo(1);
    while (robot.catapultLoaded() && opModeIsActive()) // run catapult until limit switch has been opened (i.e. ball has been launched)
      robot.runChoo(1);
    while (!robot.catapultLoaded() && opModeIsActive()) // load the catapult for next shot.
      robot.runChoo(1);
    robot.runChoo(0); // stop the catapult.
  }

  /**
   * Moves the robot in a straight line using the IMU, for a specified number of encoder ticks.
   *
   * @param pow motor power
   * @param angle angle to move at
   * @param ticks number of ticks to move
   * @param timeout milliseconds to move before stopping
   */
  void moveTicks(double pow, double angle, int ticks, int timeout) {
    robot.resetTicks();
    robot.imu.update();
    double h = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    long currentTime = startTime;
    // While we still have ticks to drive AND we haven't exceeded the time limit, move in the specified direction.
    while (Math.abs(robot.getTicks()) < ticks && currentTime - startTime < timeout && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(Math.abs(pow), angle, robot.imu.heading(), h);
      currentTime = System.currentTimeMillis();
      telemetry.addData("Target", ticks);
      telemetry.addData("Current", robot.getTicks());
      telemetry.update();
      manageAsync();
    }
    robot.stopMotors();
  }

  /**
   * Drive forward or backward a specified number of encoder ticks.
   * @param pow SPEED
   * @param ticks number of ticks forward
   * @param timeout seconds before you stop moving if encoders don't finish
   */
  void driveTicks(double pow, int ticks, int timeout) {
    double angle = pow > 0 ? 0 : Math.PI;
    moveTicks(pow, angle, ticks, timeout);
  }

  /**
   * Moves the robot in a straight line using the IMU, for a specified number of encoder ticks.
   *
   * @param pow motor power
   * @param angle angle to move at
   * @param ticks number of ticks to move
   */
  void moveTicks(double pow, double angle, int ticks) {
    moveTicks(pow, angle, ticks, 30000);
  }

  /**
   * Drive forward or backward a specified number of encoder ticks.
   * @param pow SPEED
   * @param ticks number of ticks forward
   */
  void driveTicks(double pow, int ticks) {
    driveTicks(pow, ticks, 30000);
  }

  /**
   * Rotate a specified number of degrees using the IMU
   * @param pow motor power
   * @param degs degrees to turn
   * @param fallbackTicks encoder ticks to fall back on if IMU fails
   * @return true if fallback was engaged.
   */
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
      manageAsync();
    } while (Math.abs(robot.imu.heading()) < degs && opModeIsActive());

//    while (b && Math.abs(robot.getTicks()) < fallbackTicks && opModeIsActive()) {
//      robot.move(0, 0, pow);
//      telemetry.addData("ticks", robot.getTicks());
//      telemetry.update();
//    }

    robot.stopMotors();
    return !b;
  }

  /**
   * Rotate a specified number of degrees using the IMU
   * @param pow motor power
   * @param degs degrees to turn
   * @return
   */
  boolean rotateDegs(double pow, double degs) {
    return rotateDegs(pow, degs, 0);
  }

  /**
   * Shoot particles, depending on the values set in the settings file.
   * @throws InterruptedException
   */
  void shootParticles() throws InterruptedException {
    if (settings.numShots > 0) { // shoot first particle if settings has us do that.
      fireParticle();
//      sleep(SLEEP_TIME);
    }
    if (settings.numShots > 1) { // if settings calls for a second particle,
      transferParticle(); // transfer it to catapult,
      sleep(SLEEP_TIME); // let the ball settle,
      fireParticle(); // shoot again.

    }
  }

  /**
   * Rotate until the two distance sensors return readings within 1 inch of each other,
   * which means the robot is aligned parallel to the wall.
   * @throws InterruptedException
   */
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
      manageAsync();
    } while (Math.abs(frontDist - backDist) > 1.0 && opModeIsActive());
    robot.stopMotors();
  }

  void initHeading() throws InterruptedException {
    robot.imu.update();
    while (Math.abs(robot.imu.rawHeading() - getDir() == 1 ? 180 : 0) > 5 && opModeIsActive()){
      robot.imu.update();
      robot.move(0, 0, ROTATE_SPEED * -FtcUtil.sign(robot.imu.rawHeading()));
    }
    robot.stopMotors();
  }

  /**
   * Move each side of the robot independently to align perpendicular to the line on the mat.
   * [Still in testing, DOESN'T WORK]
   * @throws InterruptedException
   */
  void alignWithLine() throws InterruptedException {
    double dir = -1;

    while (!robot.isOnLinel() && opModeIsActive()) {
      robot.drive(SPEED*dir, SPEED*dir, 0, 0);
    }
    while  (!robot.isOnLiner() && opModeIsActive()) {
      robot.drive(0, 0, SPEED*dir, SPEED*dir);
    }

    robot.stopMotors();
  }

  /**
   * Sense the colors for each side of the beacon and push the correct one.
   * @param color 1 if red, -1 if blue.
   * @param second false if first beacon, true if second beacon. The robot pushes into the beacon differently depending on which beacon it is.
   * @return
   * @throws InterruptedException
   */
  public int pushButton (int color, boolean second) throws InterruptedException {
    int redLeft, blueLeft;
    robot.colorSensor.enableLed(false);
    redLeft = robot.colorSensor.red();
    blueLeft = robot.colorSensor.blue();
    print("Red:" + redLeft + ", Blue: " + blueLeft);
    int hit = 0;
    if(redLeft*color > blueLeft*color) {
      if (second) pushSecond();
      else pushFirst();
      hit = 1;
//      sleep(500);
    }

    if (hit==0) { // if we didn't hit the first side, drive forward and sense the next side.
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

  /**
   * Procecure for hitting the first beacon.
   * 1) Push out button hitter
   * 2) Strafe into beacon
   * 3) Retract button hitter and strafe away from beacon to align for second beacon.
   */
  void pushFirst() {
    robot.pushOut();
    sleep(2000);
    robot.pushStop();
//    sleep(200);
    moveTicks(STRAFE_SPEED, -Math.PI / 2, 500, 500);
    sleep(200);
    asyncPushIn(2000);
    moveTicks(STRAFE_SPEED, Math.PI / 2, 700, 1000);
  }

  /**
   * Procedure for hitting the second beacon.
   * 1) Push out button hitter
   * 2) Strafe into beacon
   * 3) Move backwards slightly in preparation for possibly knocking off cap ball.
   */
  void pushSecond() {
    robot.pushOut();
    sleep(2000);
    robot.pushStop();
//    sleep(200);
    moveTicks(.8, -Math.PI / 2, 500, 500);
    sleep(200);
    asyncPushIn(2000);
    moveTicks(.8, Math.PI / 2, 250, 500);
//    moveTicks(STRAFE_SPEED, Math.PI / 2, 500, 500);
  }

  /**
   * Procedure for approaching beacon.
   * 1) Make sure line sensors are on proper side of line
   * 2) Align on the line in front of the beacon
   * 3) Drive a number of ticks to line up with first side of beacon.
   */
  void approachBeacon() {
    if (getDir() > 0) {
//      print("going forward...");
//      driveTicks(SPEED * getDir(), 300);
//      robot.stopMotors();
//      sleep(SLEEP_TIME);
      print("finding line...");
      moveUntilOnLine(SPEED / 2, BACKWARD);
      print("line found.");
      sleep(SLEEP_TIME);
      // drive forward to align with beacon
      print("drive forwards");
      driveTicks(SPEED / 2 * getDir(), 125);
    } else if (getDir() < 0) {
      moveUntilOnLine(SPEED / 2, BACKWARD);
      sleep(SLEEP_TIME);
      driveTicks(SPEED, 390);
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
      manageAsync();
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
      manageAsync();
    }
    robot.stopMotors();
  }
  void print(String s) {
    telemetry.addData(">", s);
    telemetry.update();
  }
}
