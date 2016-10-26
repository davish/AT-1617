package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

  String FIRST_TARGET = "legos";
  String SECOND_TARGET = "tools";

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    // Move forward to get in range of vision targets
    robot.imu.resetHeading();
    do {
      robot.imu.update();
      robot.moveStraight(SPEED, 0, robot.imu.heading());
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    // Keep moving until we're 63cm away from the target
    double orientationDiscrepancy = Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET));
    sleep(500);
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 700 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(SPEED, 0, robot.imu.heading());
      idle();
    }
    robot.stopMotors();
    sleep(500);
    robot.imu.resetHeading();
    do {
      robot.imu.update();
      robot.move(0, 0, SPEED);
      idle();
    } while (orientationDiscrepancy + robot.imu.heading() > 5 && opModeIsActive());
    robot.stopMotors();

    sleep(500);
    robot.imu.resetHeading();
    do {
      robot.imu.update();
      robot.move(SPEED, -Math.PI/2, robot.imu.heading());
    } while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]) > 50);

    sleep(500);
  }
}