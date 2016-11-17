package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;

/**
 * Created by davis on 10/20/16.
 */
@Autonomous(name="Hit Red Button", group="auto")
public class HitButton extends LinearOpMode {
  public void runOpMode() throws InterruptedException {
    Holonomic robot = new Mecanum();
    robot.init(hardwareMap);
    waitForStart();
    robot.hitBeacon(1);
    sleep(1000);
  }
}
