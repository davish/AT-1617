package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;


/**
 * Created by student on 11/22/16.
 */
@Disabled
@Autonomous(name = "lineFollow 2")
public class LineFollowerTest extends LinearOpMode {

    Orion robot = new Orion();

    public void runOpMode () throws InterruptedException
    {
        robot.init(hardwareMap);
        telemetry.addData("status", "robotinitialized");
        telemetry.update();

        waitForStart();

//        if((robot.distl.getVoltage() > .5) && (robot.distr.getVoltage() > .5)) {
//            while (opModeIsActive()) {
//                robot.lineFollow();
//            }
//        }


    }
}
