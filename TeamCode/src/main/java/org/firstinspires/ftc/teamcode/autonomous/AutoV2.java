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
@Autonomous(name="Diagonal", group="tests")
public class AutoV2 extends AutoBase{
  double power = .4;
  int color = 0;
  double getDir() {
    return 1;
  }
  public void run() throws InterruptedException {

    driveTicks(-power, 1250); // drive forward to shoot
    sleep(SLEEP_TIME);
    shootParticles();
    rotateDegs(power, 110);
    sleep(SLEEP_TIME);
    driveTicks(power, 1000);
    rotateDegs(power, 60);
    while(!robot.isOnLinel() && opModeIsActive()){
      driveTicks(power, 350);
    }
    robot.stopMotors();
    rotateDegs(power, 70);
    sleep(SLEEP_TIME);
    alignWithLine();
    sleep(SLEEP_TIME);
    while (robot.getDistanceAway() > 10) {
      robot.move(1.0, Math.PI/2, 0);
    }
    robot.stopMotors();
    sleep(SLEEP_TIME);

    pushButton((int) getDir());
  }
}
