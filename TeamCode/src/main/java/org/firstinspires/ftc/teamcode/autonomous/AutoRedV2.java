package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.chassis.Orion;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

import java.io.File;
import java.io.IOException;

/**
 * Created by davis on 1/6/17.
 */
@Autonomous(name="Red Auto v2", group="tests")
public class AutoRedV2 extends AutoBase{
  double power = .3;
  int color = 0;
  double getDir() {
    return 1;
  }
  public void run() throws InterruptedException {
    robot.colorSensor.enableLed(false);
    driveTicks(-power, 1250); // drive forward to shoot
    sleep(SLEEP_TIME);
//    shootParticles();
    rotateDegs(power, 110); // rotate and move towards beacon
    sleep(SLEEP_TIME);
    driveTicks(power, 2300);
    sleep(SLEEP_TIME);
    rotateDegs(power, 45); // rotate into alignment with wall
    sleep(SLEEP_TIME);
    print("strafe");
    moveUntilCloserThan(15, .8); // strafe until we're within pushing range
    sleep(SLEEP_TIME*2);
    // move backwards until we're aligned with the line.
    print("move backwards");
    moveUntilOnLine(power/2, BACKWARD);
    sleep(SLEEP_TIME*3);
    // drive forward to align with beacon, then push the proper button
    print("drive forwards");
    driveTicks(power/2, 200);
    pushButton((int)getDir()); // code to push beacon
    sleep(SLEEP_TIME);

    if (settings.beacon2) {
      driveTicks(power, 1800); // go with encoders fast until we're approximately to the line, then
      moveUntilOnLine(power/2, FORWARD); // stop when we see the line,
      sleep(SLEEP_TIME*2);
//      moveUntilCloserThan(20, .8); // make sure we're still in range to push
      sleep(SLEEP_TIME*2);
      // because of slop we're going to overshoot the line, so move backwards until we see the line again.
      moveUntilOnLine(power/2, BACKWARD);
      sleep(SLEEP_TIME);
      driveTicks(power/2, 200); // move to align with beacon for first hit
      sleep(SLEEP_TIME);
      pushButton((int) getDir());

      //cap ball
      if(settings.knockCapBall)
      {
        rotateDegs(power, 150);
        driveTicks(power, 2500); //figure out how many ticks
      }

    }
    else if (settings.knockCapBall){
      rotateDegs(power, 90);
      driveTicks(power, 2000);
      rotateDegs(power, 85);
    }
  }
}
