package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.chassis.*;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

/**
 * Created by davis on 9/29/16.
 */
@Autonomous(name="Line to Gears", group="Tests")
public class StraightLine extends LinearOpMode{
  Holonomic robot;

  double SPEED = .15;
  String TARGET = "gears";

  public void runOpMode() throws InterruptedException{
    robot = new Omni();
    robot.init(hardwareMap);
    Vuforia vuforia = new Vuforia();
    waitForStart();
    vuforia.activate();
    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment(TARGET);
      // pos={x, y z}. y is alignment, z is distance from target. x is height, which doesn't matter as of now.
      float[] pos = vuforia.getPosition(loc);
      float heading = vuforia.getHeading(loc); // heading is rotation around (y) axis.

      double pow = 0, angle = 0, rot = 0;

      if (Math.abs(pos[2]) > 500) {
        angle = Math.atan2(pos[2], pos[1]) + Math.PI/2;
        pow = SPEED;
      } else {
        pow = 0;
        angle = 0;
      }


      if (!Float.isNaN(heading))
        robot.moveStraight(pow, angle, heading);
      else
        robot.move(0, 0, 0);
    }
  }
}
