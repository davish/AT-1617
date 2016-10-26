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
@Autonomous(name="3 Updated Original Auto", group="auto")
public class NewRedAuto extends LinearOpMode {
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
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 630 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(SPEED, 0, robot.imu.heading());
      idle();
    }
    robot.stopMotors();
    sleep(500);
    // Rotate until we're facing the target
    while (Math.abs(Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET))) > 5 && opModeIsActive()) {
      robot.move(0, 0, SPEED);
      idle();
    }
    robot.stopMotors();
    sleep(500);

    // Strafe/move forward until we're 30cm away.
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 300 && opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment(FIRST_TARGET);
      robot.alignWithTarget(Vuforia.getPosition(loc), Vuforia.getHeading(loc), .8);
    }
    robot.stopMotors();
    sleep(500);
  }
}