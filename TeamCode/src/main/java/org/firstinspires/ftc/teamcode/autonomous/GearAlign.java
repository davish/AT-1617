package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.*;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Align with Gears", group="Tests")
public class GearAlign extends LinearOpMode {
  Holonomic robot;

  double SPEED = .25;
  String TARGET = "gears";

  public void runOpMode() throws InterruptedException{
    robot = new Omni();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    waitForStart();
    vuforia.activate();

    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment(TARGET);
      // pos={x, y z}. x is alignment, z is distance from target. y is height, which doesn't matter as of now.
      float[] pos = vuforia.getPosition(loc);
      float heading = vuforia.getHeading(loc); // heading is rotation around (y) axis.

      double pow = 0, angle = 0, rot = 0;
      if (loc == null) { // if the target can't be located, rotate slowly so we can find it.
        rot = SPEED / 2;
      }
      else if (Math.abs(pos[0]) > 10) { // Line up horizontally by strafing left or right
        pow = SPEED;
        angle = FtcUtil.sign(pos[0]) * Math.PI/2;
      }
      else if (Math.abs(heading) > 3) { // If lined up horizontally, rotate to the right orientation
        rot = FtcUtil.sign(heading) * SPEED;
      }
      else if (Math.abs(pos[2]) > 200) { // If we're lined up, move forward until we're within 20cm
        pow = SPEED;
      }
      // if we're lined up and the target's in view, don't move.

      telemetry.addData("power", pow);
      telemetry.addData("angle", angle);
      telemetry.addData("rotation", rot);
      telemetry.update();

      robot.move(pow, angle, rot);
    }
  }
}
