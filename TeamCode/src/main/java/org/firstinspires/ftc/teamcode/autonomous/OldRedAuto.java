package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

/**
 * Created by davis on 10/6/16.
 */
@Autonomous(name="2 Original Auto", group="auto")
@Disabled
public class OldRedAuto extends LinearOpMode {
  Holonomic robot;

  double SPEED = 0.6;

  String FIRST_TARGET = "wheels";
  String SECOND_TARGET = "tools";

  float PHONE_OFFSET = 30; // offset of phone camera in millimeters

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    // Move forward to get in range of vision targets
    do {
      robot.move(SPEED, 0, 0);
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    // Keep moving until we're 63cm away from the target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 630 && opModeIsActive()) {
      robot.move(SPEED, 0, 0);
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
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 300) {
      OpenGLMatrix loc = vuforia.getAlignment(FIRST_TARGET);
      robot.alignWithTarget(Vuforia.getPosition(loc), Vuforia.getHeading(loc), .8);
    }
    robot.stopMotors();
    sleep(500);
  }
}