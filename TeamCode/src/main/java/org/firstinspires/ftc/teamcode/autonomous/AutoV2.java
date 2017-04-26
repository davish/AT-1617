package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Created by davis on 2/21/17.
 */
public abstract class AutoV2 extends AutoBase{
  double WALL_DISTANCE = 7.6;
  public void run() throws InterruptedException {
    long startTime = System.currentTimeMillis();
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

      if (settings.knockCapBall) {
        robot.runPickup(-1); // brush cap ball out of the way by running nom backwards.
        sleep(SLEEP_TIME);
        driveTicks(-SPEED, CAP_BALL_TICKS);
      }
      else {
        driveTicks(SPEED, FORWARD_TICKS/2);
      }
      robot.runPickup(0);
      stop();
      return;
    }

    int fullticks = 1250;
    int shootingdist = 800;

    driveTicks(-FAST_SPEED, shootingdist); // drive 1250 forward to shoot
    sleep(SLEEP_TIME);
    shootParticles();
    driveTicks(-SPEED, fullticks - shootingdist); // drive forward some more to align with old shooting
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
      if (settings.evadeDefense) {
        moveTicks(1.0, Math.PI/2, 1800);
      }
      if (!settings.waitForDefense) {
        if (getDir() == 1)
          driveTicks(FAST_SPEED * getDir(), btn == 1 ? 2600 : 2800);
        else
          driveTicks(FAST_SPEED * getDir(), btn == 1 ? 2200 : 2490);
      } else {
        if (getDir() == 1)
          driveTicks(FAST_SPEED * getDir(), btn == 1 ? 1600 : 1800);
        else
          driveTicks(FAST_SPEED * getDir(), btn == 1 ? 1200 : 1490);

        while (System.currentTimeMillis() - startTime < 23000)
          ;

        if (getDir() == 1)
          driveTicks(FAST_SPEED * getDir(), 1000);
      }

      sleep(SLEEP_TIME);
      moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
//      if (getDir() == -1) driveTicks(-SPEED, 200);
      approachBeacon();
      sleep(SLEEP_TIME);
      pushButton((int) getDir(), true);
      if (settings.knockCapBall) {
        rotateDegs(-ROTATE_SPEED, getDir() == 1 ? 40 : 140);
        robot.runPickup(-1);
        driveTicks(-FAST_SPEED, 3500);
        robot.runPickup(0);
      }
      else if (settings.endOnCenter) {
        moveTicks(FAST_SPEED, Math.PI/2, 900);
//        alignWithWall();
        driveTicks(1.0 * getDir(), getDir() == 1 ? 4600 : 4800);
      }

    }
  }
}
