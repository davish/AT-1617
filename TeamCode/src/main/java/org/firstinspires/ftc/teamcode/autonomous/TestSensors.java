package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.chassis.Atlas;
import org.firstinspires.ftc.teamcode.chassis.Orion;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Test Sensors", group="test")
public class TestSensors extends LinearOpMode{
  public void runOpMode() throws InterruptedException {
    Atlas robot = new Atlas();
    robot.init(hardwareMap);

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Limit switch", robot.catapultLoaded());
//
//      robot.imu.update();
//      telemetry.addData("Absolute Orientation", robot.imu.heading());
//      telemetry.addData("Line", robot.odsl.getLightDetected());
//
//      telemetry.addData("Red", robot.colorSensor.red());
//      telemetry.addData("Blue", robot.colorSensor.blue());
////      telemetry.addData("Sonar Distance", robot.distr.getVoltage());
      telemetry.update();
    }
  }
}
