package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.chassis.Atlas;
import org.firstinspires.ftc.teamcode.chassis.Orion;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

@Autonomous(name="Test Sensors", group="test")
public class TestSensors extends LinearOpMode{
  public void runOpMode() throws InterruptedException {
    Atlas robot = new Atlas();
    robot.init(hardwareMap);
    telemetry.addData(">", "Init complete");
    waitForStart();
  int i = 0;
    while (opModeIsActive()) {
//      robot.imu.update();
//      telemetry.addData("Limit switch", robot.catapultLoaded());
//      telemetry.addData("Heading", robot.imu.heading());
//      telemetry.addData("Left Line", robot.isOnLinel());
//      telemetry.addData("Red", robot.colorSensor.red());
//      telemetry.addData("Blue", robot.colorSensor.blue());

      robot.dist.setMode(DigitalChannelController.Mode.OUTPUT);
      robot.dist.setState(false);
      Thread.sleep(0, 2000);
      robot.dist.setState(true);
      Thread.sleep(0, 5000);
      robot.dist.setState(false);
      robot.dist.setMode(DigitalChannelController.Mode.INPUT);
      long m = System.nanoTime();
      while (robot.dist.getState() && opModeIsActive())
        ;
      long n = System.nanoTime() - m;
      telemetry.addData("Distance", n);
      telemetry.addData("Ping", i++);
      telemetry.update();
      sleep(500);
    }
  }
}
