package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Created by davis on 2/21/17.
 */
public abstract class AutoV2 extends AutoBase{
  public void run() throws InterruptedException {
    robot.colorSensor.enableLed(false);
    driveTicks(-SPEED * getDir(), 1250); // drive forward to shoot
    sleep(SLEEP_TIME);
    shootParticles();
    rotateDegs(SPEED, getDir() == 1 ? 110 : 70); // rotate and move towards beacon
    sleep(SLEEP_TIME);
    driveTicks(SPEED * getDir(), 2300);
    sleep(SLEEP_TIME);
    rotateDegs(SPEED, 45); // rotate into alignment with wall
    sleep(SLEEP_TIME);
    print("strafe");
    moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
    sleep(SLEEP_TIME * 2);
    approachBeacon();
    sleep(SLEEP_TIME);
    pushButton((int)getDir());
    sleep(SLEEP_TIME);

    if (settings.beacon2) {
      driveTicks(FAST_SPEED, 1800); // go with encoders fast until we're approximately to the line, then
      moveUntilOnLine(SPEED / 2, getDir() == 1 ? FORWARD: BACKWARD); // stop when we see the line,
      sleep(SLEEP_TIME*2);
      approachBeacon();
      sleep(SLEEP_TIME);
      pushButton((int) getDir());
    }
  }
}
