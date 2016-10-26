package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

/**
 * Created by davis on 10/6/16.
 */
@Autonomous(name="1 Red Auto", group="auto")
public class RedAutonomous extends LinearOpMode{
  Holonomic robot;

  double SPEED = .8;

  String FIRST_TARGET = "wheels";
  String SECOND_TARGET = "tools";


  OpenGLMatrix loc;
  float[] pos;
  float heading;
  double gyroHeading;
  Vuforia vuforia;

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    // Move forward to get in range of vision targets
    telemetry.addData(">", "move blindly");
    telemetry.update();

    updatePosition(FIRST_TARGET);
    double straightAhead = gyroHeading;
    while (opModeIsActive()) {
      updatePosition(FIRST_TARGET);
      robot.moveStraight(SPEED, 0, gyroHeading, straightAhead);
      if (loc != null) {
        robot.stopMotors();
        break;
      }
      idle();
    }
    // Keep moving until we're 63cm away from the target
    robot.stopMotors();
    telemetry.addData(">", "get 63 cm away");
    telemetry.update();
    sleep(1000);

    straightAhead = heading;
    while (opModeIsActive()) {
      updatePosition(FIRST_TARGET);
      robot.moveStraight(SPEED, 0, heading, straightAhead);
      if (Math.abs(pos[2]) <= 630) {
        robot.stopMotors();
        break;
      }
      idle();
    }

    telemetry.addData(">", "align with wall");
    telemetry.update();
    sleep(1000);

    updatePosition(FIRST_TARGET);
    robot.move(0, 0, SPEED * -FtcUtil.sign(heading));
    robot.imu.resetHeading();
    double turnDegs = heading;
    while (opModeIsActive()) {
      updatePosition(FIRST_TARGET);
      // once we've turned w/ gyro the right number of degrees,
      if (Math.abs(gyroHeading) > Math.abs(turnDegs)) {
        robot.stopMotors();
        break;
      }
      idle();
    }

    telemetry.addData(">", "strafe horizontally");
    telemetry.update();
    sleep(500);
    robot.imu.resetHeading();
    while (opModeIsActive()) {
      updatePosition(FIRST_TARGET);

      robot.moveStraight(SPEED, -Math.PI/2, gyroHeading);

      if (Math.abs(pos[1]) < 50) {
        robot.stopMotors();
        break;
      }
      idle();
    }
    robot.stopMotors();

    telemetry.addData(">", "move forward!");
    telemetry.update();
    sleep(500);

    while (opModeIsActive()) {
      updatePosition(FIRST_TARGET);
      robot.moveStraight(SPEED, 0, heading);
      if (Math.abs(pos[2]) < 300) {
        robot.stopMotors();
        break;
      }
      idle();
    }
    robot.stopMotors();
    telemetry.addData(">", "puuush!");
    telemetry.update();
    sleep(500);
    robot.hitBeacon(1);
    sleep(1000);
  }

  void updatePosition(String target) {
    loc = vuforia.getAlignment(target);
    pos = Vuforia.getPosition(loc);
    heading = Vuforia.getHeading(loc);
    robot.imu.update();
    gyroHeading = robot.imu.heading();
  }
  void updatePosition() {
    updatePosition(FIRST_TARGET);
  }
}
