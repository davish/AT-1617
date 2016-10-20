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
@Autonomous(name="Red Auto", group="auto")
public class RedAutonomous extends LinearOpMode{
  Holonomic robot;

  double SPEED = 0.6;

  String FIRST_TARGET = "gears";
  String SECOND_TARGET = "tools";


  OpenGLMatrix loc;
  float[] pos;
  float heading;
  double distance;
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
    do {
      robot.move(SPEED, 0, 0);
      updatePosition(FIRST_TARGET);
      idle();
    } while (loc == null && opModeIsActive());
    // Keep moving until we're 63cm away from the target
    robot.stopMotors();
    sleep(1000);
    telemetry.addData(">", "get 63 cm away");
    telemetry.update();
    updatePosition();
    while (Math.abs(pos[2]) > 630 && opModeIsActive()) {
      robot.move(SPEED, 0, 0);
      updatePosition(FIRST_TARGET);
      idle();
    }
    robot.stopMotors();
    sleep(500);
    // Rotate until we're facing the target
    telemetry.addData(">", "align with wall");
    telemetry.update();
    updatePosition();
    while (Math.abs(heading) > 5  && opModeIsActive()) {
      robot.move(0, 0, SPEED*-FtcUtil.sign(heading));
      updatePosition(FIRST_TARGET);
      idle();
    }
    robot.stopMotors();
    sleep(500);
    telemetry.addData(">", "strafe horizontally");
    telemetry.update();
    // Strafe/move forward until we're 30cm away.
    do {
      updatePosition(FIRST_TARGET);
      if (loc != null)
        robot.alignWithTarget(pos, heading, .8);
      else
        robot.move(SPEED, Math.PI/2, 0);
    } while ((loc == null || distance > 300) && opModeIsActive());
    robot.stopMotors();
    sleep(500);
    telemetry.addData(">", "puuush!");
    telemetry.update();
//    robot.hitBeacon(1);

    sleep(10000);
  }

  void updatePosition(String target) {
    loc = vuforia.getAlignment(target);
    pos = Vuforia.getPosition(loc);
    heading = Vuforia.getHeading(loc);
    distance = robot.rangeSensor.getDistance(DistanceUnit.MM);
  }
  void updatePosition() {
    updatePosition(FIRST_TARGET);
  }
}
