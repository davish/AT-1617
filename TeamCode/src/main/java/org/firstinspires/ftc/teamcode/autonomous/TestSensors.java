package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Test Sensors", group="test")
public class TestSensors extends LinearOpMode{
  public void runOpMode() throws InterruptedException {
    Vuforia vuforia = new Vuforia();
    Holonomic robot = new Mecanum();
    robot.init(hardwareMap);
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    while (opModeIsActive()) {
      OpenGLMatrix loc = vuforia.getAlignment("gears");
      float[] pos = Vuforia.getPosition(loc);
      float heading = Vuforia.getHeading(loc); // heading is rotation around (y) axis.

      robot.imu.update();
      telemetry.addData("Absolute Orientation", robot.imu.heading());
      telemetry.addData("Limit switch", robot.catapultLoaded());
      telemetry.addData("Line", robot.ods.getLightDetected());
      telemetry.addData("Alignment", pos[1]);
      telemetry.addData("Distance", pos[2]);
      telemetry.addData("Sonar Distance", robot.dist.getVoltage());
      telemetry.addData("Relative Orientation", heading);
      telemetry.update();
    }
  }
}
