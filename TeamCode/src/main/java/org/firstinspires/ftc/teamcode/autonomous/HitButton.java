package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;

/**
 * Created by davis on 10/20/16.
 */
@Autonomous(name="Hit Button", group="test")
public class HitButton extends LinearOpMode {
  public void runOpMode() throws InterruptedException {
    Orion robot = new Orion();
    robot.init(hardwareMap);
//    waitForStart();
    robot.push();
//    robot.hitBeacon(1);
//    sleep(1000);
  }
}
