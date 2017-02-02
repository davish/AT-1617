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
  double power = .3;
  int color = 0;
  double getDir() {
    return 1;
  }
  public void run() throws InterruptedException {

    driveTicks(-power, 1250); // drive forward to shoot
    sleep(SLEEP_TIME);
//    shootParticles();
    rotateDegs(power, 110);
    sleep(SLEEP_TIME);
    driveTicks(power, 2300);
    sleep(SLEEP_TIME);
    rotateDegs(power, 45);
    sleep(SLEEP_TIME);
//    alignWithLine();
//    sleep(SLEEP_TIME);
    robot.imu.update();
    double h = robot.imu.heading();
    while (robot.getDistanceAway() > 15 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(.8, Math.PI/2, robot.imu.heading(), h);
    }
    robot.stopMotors();
    sleep(SLEEP_TIME*2);
    while (!robot.isOnLinel() && opModeIsActive()) {
      robot.move(power/2, Math.PI, 0);
    }
    robot.stopMotors();
    sleep(SLEEP_TIME);
    driveTicks(power/2, 150);
    pushButton(1);
//    driveTicks(power, 2000);
//    driveTicks(-power/2, 400);
//    robot.push();

  }
}
