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
public class DiagonalTest extends AutoBlue{
  public void runOpMode() throws InterruptedException {
    double power = .4;
    int color = 0;

    Gson gson = new Gson();
    File sfile = AppUtil.getInstance().getSettingsFile("auto_settings.json");
    try {
      settings = gson.fromJson(ReadWriteFile.readFileOrThrow(sfile), Settings.class);
    } catch (IOException e) {
      settings = new Settings();
    }
    robot = new Orion();
    robot.init(hardwareMap);
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

    driveTicks(-power, 1250);
    sleep(500);
    shootParticles();
    rotateDegs(power, 110);
    sleep(500);
    driveTicks(power, 1000);
    rotateDegs(power, 60);
    while(!robot.isOnLinel()){
      driveTicks(power, 350);
    }
    if(robot.isOnLinel()) {
     driveTicks(power, 800); // ?
      rotateDegs(power, 90);



    }

  }
}
