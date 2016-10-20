package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.*;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Align with Gears", group="Tests")
public class GearAlign extends LinearOpMode {
  Holonomic robot;

  double SPEED = 0.6;
  String FIRST_TARGET= "gears";

  OpenGLMatrix loc;
  float[] pos;
  float heading;
  double distance;
  Vuforia vuforia;

  public void runOpMode() throws InterruptedException{
    robot = new Mecanum();
    robot.init(hardwareMap);

    vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    updatePosition(FIRST_TARGET);
    telemetry.addData("distance", pos[2]);
    telemetry.update();
    do {
      updatePosition(FIRST_TARGET);
      if (loc != null)
        robot.alignWithTarget(pos, heading, .8);
      else
        robot.move(SPEED, -Math.PI/2, 0);
    } while ((loc == null || Math.abs(pos[2]) > 300) && opModeIsActive());
    robot.stopMotors();
  }
  void updatePosition(String target) {
    loc = vuforia.getAlignment(target);
    pos = Vuforia.getPosition(loc);
    heading = Vuforia.getHeading(loc);
    distance = robot.rangeSensor.getDistance(DistanceUnit.MM);
  }
}
