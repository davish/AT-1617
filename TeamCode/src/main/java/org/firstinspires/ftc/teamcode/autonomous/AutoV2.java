package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Created by davis on 2/21/17.
 */
public abstract class AutoV2 extends AutoBase{
  double WALL_DISTANCE = 7;
  public void run() throws InterruptedException {
    robot.colorSensor.enableLed(false);

    if (!settings.beacon1 && !settings.beacon2 && settings.numShots > 0) {
      int FORWARD_TICKS = 2500;
      int CAP_BALL_TICKS = 1200;
      driveTicks(-SPEED, FORWARD_TICKS);
      sleep(SLEEP_TIME*2);
      shootParticles();
      robot.runPickup(-1);
      sleep(SLEEP_TIME);

      if (settings.knockCapBall) {
        driveTicks(-SPEED, CAP_BALL_TICKS);
      }
      robot.runPickup(0);
      stop();
      return;
    }

    driveTicks(-FAST_SPEED, 1000); // drive 1250 forward to shoot
    sleep(SLEEP_TIME);
    shootParticles();
    driveTicks(-SPEED, 250);
    sleep(SLEEP_TIME);
    rotateDegs(ROTATE_SPEED, getDir() == 1 ? 110 : 45); // rotate and move towards beacon
    sleep(SLEEP_TIME);

    driveTicks(FAST_SPEED * getDir(), 2400);
    sleep(SLEEP_TIME);

    rotateDegs(ROTATE_SPEED * getDir(), getDir() == 1 ? 50 : 42); // rotate into alignment with wall
    sleep(SLEEP_TIME);

    print("strafe");
    moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
    sleep(SLEEP_TIME);
    approachBeacon();
    sleep(SLEEP_TIME);
    int btn = pushButton((int)getDir(), false);
//    sleep(SLEEP_TIME);

    if (settings.beacon2) {
      driveTicks(FAST_SPEED*getDir(), btn == 1 ? 2200 : 2490); // go with encoders fast until we're past the line, then approach beacon normally.
      moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
      approachBeacon();
      sleep(SLEEP_TIME);
      pushButton((int) getDir(), true);
      if (settings.knockCapBall) {
        robot.drive(1, 0, 0, 1);
        sleep(3000);
        robot.stopMotors();
      }
    }
  }
}
