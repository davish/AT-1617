package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Created by davis on 2/21/17.
 */
public abstract class AutoV2 extends AutoBase{
  double WALL_DISTANCE = 7.6;
  public void run() throws InterruptedException {
    robot.colorSensor.enableLed(false);

    /*
     * ALTERNATIVE AUTONOMOUS PROGRAM:
     * If no beacon is set to be hit, run this autonomous program from the corner of the field,
     * With the robot lined up to shoot in the vortex when it moves forward.
     */
    if (!settings.beacon1 && !settings.beacon2 && settings.numShots > 0) {
      int FORWARD_TICKS = 2500;
      int CAP_BALL_TICKS = 1200;
      driveTicks(-SPEED, FORWARD_TICKS);
      sleep(SLEEP_TIME*2);
      shootParticles();
      robot.runPickup(-1); // brush cap ball out of the way by running nom backwards.
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
    driveTicks(-SPEED, 250); // drive forward some more to align with old shooting
    sleep(SLEEP_TIME);
    rotateDegs(ROTATE_SPEED, getDir() == 1 ? 110 : 45); // rotate and move towards beacon
    sleep(SLEEP_TIME);

    driveTicks(FAST_SPEED * getDir(), 2400); // sprint towards beacon
    sleep(SLEEP_TIME);

    rotateDegs(ROTATE_SPEED * getDir(), getDir() == 1 ? 50 : 42); // rotate into alignment with wall
    sleep(SLEEP_TIME);

//    alignWithWall(); // align using ultrasonic sensors
    initHeading();

    print("strafe");
    moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
    sleep(SLEEP_TIME);
    approachBeacon();
    sleep(SLEEP_TIME);
    int btn = pushButton((int)getDir(), false);
//    sleep(SLEEP_TIME);

    if (settings.beacon2) {
      // go with encoders fast until we're past the line, then approach beacon normally.
      // Depending on what side of the button was hit, use different encoder values.
      if (getDir() == 1)
        driveTicks(FAST_SPEED*getDir(), btn == 1 ? 2500 : 2800);
      else
        driveTicks(FAST_SPEED*getDir(), btn == 1 ? 2200 : 2490);
      sleep(SLEEP_TIME);
      moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
      approachBeacon();
      sleep(SLEEP_TIME);
      pushButton((int) getDir(), true);
      if (settings.knockCapBall) {
        rotateDegs(-ROTATE_SPEED * getDir(), getDir() == 1 ? 45 : 30);
        driveTicks(-FAST_SPEED * getDir(), 3500);
      }
      else if (settings.endOnCenter) {
        moveTicks(FAST_SPEED, Math.PI/2, 900);
//        alignWithWall();
        driveTicks(1.0 * getDir(), getDir() == 1 ? 4600 : 4800);
      }
    }
  }
}
