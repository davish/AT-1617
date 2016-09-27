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
  public void runOpMode() throws InterruptedException{

    double SPEED = .25;

    robot = new Omni();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    waitForStart();
    vuforia.activate();

    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment("gears");
      // pos={x, y z}. x is alignment, z is distance from target. y is height, which doesn't matter as of now.
      float[] pos = vuforia.getPosition(loc);
      float heading = vuforia.getHeading(loc); // heading is rotation around (y) axis.

      if (loc == null) { // if the target can't be located, rotate slowly.
        robot.omniMove(0, 0, SPEED/2);
      }
      else if (Math.abs(pos[0]) > 10) { // Line up horizontally by strafing left or right
        robot.omniMove(SPEED, FtcUtil.sign(pos[0])*Math.PI/4, 0);
      }
      else if (Math.abs(heading) > 3) { // If lined up horizontally, rotate to the right orientation
        robot.omniMove(0, 0, FtcUtil.sign(heading) * SPEED);
      }
      else if (Math.abs(pos[2]) > 200) { // If we're lined up, move forward until we're within 20cm
        robot.omniMove(SPEED, 0, 0);
      }
      else { // if we're lined up and the target's in view, don't move.
        robot.stopMotors();
      }
    }
  }
}
