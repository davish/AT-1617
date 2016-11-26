package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Mecanum;

/**
 * Created by student on 11/22/16.
 */
 @Autonomous(name = "lineFollow") public class StrafeTest extends LinearOpMode {

    Mecanum robot = new Mecanum();
    double distance;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("status", "robotinitialized");
        telemetry.update();

        waitForStart();

        while (robot.ods.getLightDetected() < .5) {
            robot.imu.update();
            robot.imu.heading();
            if (robot.dist.getVoltage() == distance) {
                robot.move(.25, (-1*Math.PI)/2, 0);

                robot.moveLeft(.25);
            } else if (robot.dist.getVoltage() > distance) {
                robot.driveTank(.15, .15);

            } else if (robot.dist.getVoltage() < distance) {
                robot.driveTank(-.15, -.15);
            }
        }
    }
}
