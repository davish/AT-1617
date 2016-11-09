package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

/**
 * Created by davis on 10/6/16.
 */
@Autonomous(name="4 Integrated Auto", group="auto")
public class SuperNewRedAuto extends LinearOpMode {
  Holonomic robot;

  double SPEED = 0.6;

  String FIRST_TARGET = "tools";
  String SECOND_TARGET = "tools";

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    robot.move(SPEED, 0, 0);
    sleep(400);
    robot.stopMotors();
    sleep(500);
    do {
      robot.imu.update();
      robot.move(0, 0, SPEED);
    } while (Math.abs(robot.imu.heading()) < 50 && opModeIsActive());
    robot.stopMotors();
    sleep(500);
    // Move forward to get in range of vision targets
    double target = robot.imu.heading();
    do {
      robot.imu.update();
      robot.moveStraight(SPEED, 0, robot.imu.heading(), target);
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    // Keep moving until we're 63cm away from the target
    sleep(500);
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 700 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(SPEED, 0, robot.imu.heading(), target);
      idle();
    }
    robot.stopMotors();
    sleep(500);
    do {
      robot.imu.update();
      robot.move(0, 0, SPEED);
    } while (Math.abs(robot.imu.heading()) < 86 && opModeIsActive());
    robot.stopMotors();
    telemetry.addData(">", robot.imu.heading());
    telemetry.update();

    sleep(500);
    robot.imu.update();
    double h = robot.imu.heading();
    while (vuforia.getAlignment(FIRST_TARGET) == null)
      ;
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 480 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, 0, robot.imu.heading(), h);
      idle();
    }
    robot.stopMotors();
    sleep(500);

    double dir = FtcUtil.sign(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]);

    while (robot.ods.getLightDetected() < 0.5  && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(1.0, Math.PI/2* dir, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 225 && opModeIsActive()) {
      robot.moveStraight(2 * SPEED / 3, 0, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    int a = robot.hitBeacon(-1);
    sleep(1000);
    int count = 0;
    if (a != 0) {
      while (opModeIsActive() && count < 2) {
        robot.move(1 * SPEED / 3, 0, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        robot.move(1 * SPEED / 3, Math.PI, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        count++;
      }
    }

    sleep(500);
    robot.imu.update();
    h = robot.imu.heading();

    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) < 480 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, Math.PI, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
      idle();
    }
    robot.stopMotors();
    sleep(500);
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]) < 250 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(1.0, Math.PI / 2 * dir, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
      idle();
    }
    robot.stopMotors();
    sleep(500);
    while (robot.ods.getLightDetected() < 0.5  && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(1.0, Math.PI/2* dir, robot.imu.heading(), h);
    }
    robot.stopMotors();
    sleep(500);
    while (robot.ods.getLightDetected() < 0.5  && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(1.0, -Math.PI/2* dir, robot.imu.heading(), h);
    }
    robot.stopMotors();

  }
}