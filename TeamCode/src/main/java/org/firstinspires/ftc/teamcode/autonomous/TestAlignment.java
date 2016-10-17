package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Test Vuforia Util", group="Tests")
public class TestAlignment extends LinearOpMode{
  public void runOpMode() throws InterruptedException {
    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment("legos");
      float[] pos = vuforia.getPosition(loc);
      float heading = vuforia.getHeading(loc); // heading is rotation around (y) axis.

      telemetry.addData("Alignment", pos[1]);
      telemetry.addData("Orientation", heading);
      telemetry.addData("Distance", pos[2]);
      telemetry.update();
    }
  }
}
