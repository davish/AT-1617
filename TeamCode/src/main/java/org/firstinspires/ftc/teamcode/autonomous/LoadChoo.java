package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;

/**
 * Created by davis on 11/18/16.
 */

@Autonomous(name="Prime Catapult", group="prep")
public class LoadChoo extends LinearOpMode{

  public void runOpMode() throws InterruptedException {
    Orion robot = new Orion();
    robot.init(hardwareMap);
    while (!robot.catapultLoaded())
      robot.runChoo(1);
    robot.runChoo(0);
  }

}
