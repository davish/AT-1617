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

  double SPEED = 0.6;
  String TARGET = "legos";

  public void runOpMode() throws InterruptedException{
    robot = new Mecanum();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment(TARGET);
      // pos={x, y z}. x is alignment, z is distance from target. y is height, which doesn't matter as of now.
      float[] pos = Vuforia.getPosition(loc);
      float heading = Vuforia.getHeading(loc); // heading is rotation around (y) axis.

      double pow = 0, angle = 0, rot = 0;

//      if (Math.abs(heading) > 14) {
//        pow = 0;
//        telemetry.addData("status", "aligning");
//      }
//      else
      if (Math.abs(pos[1]) > 50) {
        pow = SPEED;
        angle = FtcUtil.sign(pos[1]) * -Math.PI/2;
        telemetry.addData(">", "strafing");
      }
      else if (Math.abs(pos[2]) > 400) {
        angle = 0;
        pow = SPEED;
        telemetry.addData(">", "approaching");
      } else {
        pow = 0;
        telemetry.addData(">", "stopped");
      }

      telemetry.addData("power", pow);
      telemetry.addData("angle", Math.toDegrees(angle));
      telemetry.update();

      if (!Float.isNaN(heading))
        robot.moveStraight(pow, angle, heading);
      else
        robot.move(0, 0, 0);
    }
  }
}
