package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
/**
 * Created by davis on 9/23/16.
 */
@Autonomous(name="Align with Gears", group="Tests")
public class GearAlign extends LinearOpMode {
  HardwareMecanum robot;
  public void runOpMode() throws InterruptedException{
    robot = new HardwareMecanum();
    robot.init(hardwareMap);
    Vuforia vuforia = new Vuforia();
    waitForStart();
    vuforia.activate();
    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment("gears");
      float[] pos = vuforia.getPosition(loc); // {x, y z}. x is alignment, z is distance from target.
      float heading = vuforia.getHeading(loc);

      if (Math.abs(pos[0]) > 10) { // Line up horizontally by strafing left or right
        robot.omniMove(.5, FtcUtil.sign(pos[0])*Math.PI/4, 0);
      } else if (Math.abs(heading) > 3) { // If lined up horizontally, rotate to the right position
        robot.omniMove(.5, 0, FtcUtil.sign(heading) * .5);
      } else if (Math.abs(pos[2]) > 200) { // If we're lined up, move forward until we're within 20cm
        robot.omniMove(.5, 0, 0);
      }
    }
  }
}
