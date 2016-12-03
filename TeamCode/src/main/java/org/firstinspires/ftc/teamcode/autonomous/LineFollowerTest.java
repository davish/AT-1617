package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Mecanum;

/**
 * Created by student on 11/22/16.
 */
@Autonomous(name = "lineFollow") public class LineFollowerTest extends LinearOpMode {

    Mecanum robot = new Mecanum();

    public void runOpMode () throws InterruptedException
    {
        robot.init(hardwareMap);
        telemetry.addData("status", "robotinitialized");
        telemetry.update();

        waitForStart();

        if((robot.distl.getVoltage() > .5) && (robot.distr.getVoltage() > .5)) {
            while (opModeIsActive()) {
                robot.lineFollow();
            }
        }


    }
}
