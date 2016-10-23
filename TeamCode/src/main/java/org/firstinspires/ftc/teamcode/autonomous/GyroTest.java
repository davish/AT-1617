package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;

/**
 * Created by davis on 10/23/16.
 */
@Autonomous(name="Straight Line", group="Tests")
public class GyroTest extends LinearOpMode{
  Holonomic robot;
  double SPEED = .8;

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    telemetry.addData(">", "Robot Initialized.");
    telemetry.update();

    waitForStart();
    robot.imu.resetHeading();
    robot.moveStraight(SPEED, 0, robot.imu.heading());
  }
}
