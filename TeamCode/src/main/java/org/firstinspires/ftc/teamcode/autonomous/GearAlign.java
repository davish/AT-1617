package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.*;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Align with Gears", group="Tests")
@Disabled
public class GearAlign extends LinearOpMode {
  Holonomic robot;

  double SPEED = 0.4;
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
    double prevReading = 0.0;

    updatePosition(FIRST_TARGET);
    robot.lineSensor.getVoltage();
    telemetry.update();
    robot.move(SPEED, -Math.PI/2, 0);
    while (opModeIsActive()) {
      telemetry.addData(">", robot.lineSensor.getVoltage());
      telemetry.update();
      double currReading = robot.lineSensor.getVoltage();
      if (currReading > .001 && prevReading > .001) {
        robot.stopMotors();
        break;
      }
      prevReading = currReading;
    }
    sleep(1000);
  }
  void updatePosition(String target) {
    loc = vuforia.getAlignment(target);
    pos = Vuforia.getPosition(loc);
    heading = Vuforia.getHeading(loc);
  }
}
