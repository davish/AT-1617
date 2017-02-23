package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 2/3/17.
 */
@Disabled
@Autonomous(name="Blue Auto v2", group="tests")
public class AutoBlueV2 extends AutoBase {
  double WALL_DISTANCE = 7;
  double getDir() {
    return 1;
  }
  public void run() throws InterruptedException {
    robot.colorSensor.enableLed(false);
    driveTicks(-SPEED, 1250); // drive forward to shoot
    sleep(SLEEP_TIME);
    shootParticles();
    rotateDegs(SPEED, 70); // rotate and move towards beacon
    sleep(SLEEP_TIME);
    driveTicks(SPEED, 2300);
    sleep(SLEEP_TIME);
    rotateDegs(SPEED, 45); // rotate into alignment with wall
    sleep(SLEEP_TIME);
    print("strafe");
    moveUntilCloserThan(WALL_DISTANCE, .8); // strafe until we're within pushing range
    sleep(SLEEP_TIME*2);
    // move backwards until we're aligned with the line.
    print("move backwards");
    moveUntilOnLine(SPEED / 2, BACKWARD);
    sleep(SLEEP_TIME*3);
    // drive forward to align with beacon, then push the proper button
    print("drive forwards");
    driveTicks(SPEED/2, 200);
    pushButton((int)getDir()); // code to push beacon
    sleep(SLEEP_TIME);

    if (settings.beacon2) {
      driveTicks(SPEED, 1800); // go with encoders fast until we're approximately to the line, then
      moveUntilOnLine(SPEED/2, FORWARD); // stop when we see the line,
      sleep(SLEEP_TIME * 2);
      sleep(SLEEP_TIME*2);
      // because of slop we're going to overshoot the line, so move backwards until we see the line again.
      moveUntilOnLine(SPEED / 2, BACKWARD);
      sleep(SLEEP_TIME);
      driveTicks(SPEED/2, 200); // move to align with beacon for first hit
      sleep(SLEEP_TIME);
      pushButton((int) getDir());
    }
  }
}
