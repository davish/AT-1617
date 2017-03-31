package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Atlas;
import org.firstinspires.ftc.teamcode.chassis.Orion;

/**
 * Created by davis on 11/18/16.
 */

@Autonomous(name="Prime Catapult", group="prep")
public class LoadChoo extends LinearOpMode{

  public void runOpMode() throws InterruptedException {
    Atlas robot = new Atlas();
    robot.init(hardwareMap, false);
    while (!robot.catapultLoaded())
      robot.runChoo(1);
    robot.runChoo(0);
  }

}
