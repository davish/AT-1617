package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;


/**
 * Created by davis on 11/17/16.
 */
@Autonomous(name="Test servo", group="test")
public class TestPositions extends LinearOpMode {
  public void runOpMode() throws InterruptedException {
    Orion robot = new Orion();
    robot.init(hardwareMap);
    telemetry.addData(">", "Robot Initialized");
    waitForStart();
    robot.senseLeft();
    sleep(1500);
    robot.senseRight();
    sleep(1500);
    robot.hitLeft();
    sleep(1500);
    robot.hitRight();
    sleep(1500);
    robot.centerServo();
    sleep(1500);

    while(opModeIsActive()) {
      robot.pivot(gamepad1.left_trigger);
      telemetry.addData(">", gamepad1.left_trigger);
      telemetry.update();
    }

  }
}
