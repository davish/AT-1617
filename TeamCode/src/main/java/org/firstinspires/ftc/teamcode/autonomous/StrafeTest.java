package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;


/**
 * Created by student on 11/22/16.
 */
@Disabled
 @Autonomous(name = "lineFollow") public class StrafeTest extends LinearOpMode {

    Orion robot = new Orion();
    double distance;

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("status", "robotinitialized");
        telemetry.update();

        waitForStart();

//        while ((robot.odsl.getLightDetected() < .5) && (robot.odsr.getLightDetected() < .5)) {
//            robot.imu.update();
//            robot.imu.heading();
//            if ((robot.distl.getVoltage() == distance) && (robot.distr.getVoltage() == distance)) {
//                robot.move(.25, (-1 * Math.PI) / 2, 0);
//                robot.moveLeft(.25);
//            }
//
//            //farther away
//            else if ((robot.distl.getVoltage() > distance) && (robot.distr.getVoltage() > distance)) {
//                robot.driveTank(.15, .15);
//            } else if ((robot.distl.getVoltage() > distance) && (robot.distr.getVoltage() == distance)) {
//                robot.moveLeft(.15);
//            } else if ((robot.distl.getVoltage() == distance) && (robot.distr.getVoltage() > distance)) {
//                robot.moveRight(.15);
//            }
//
//
//            //closer
//            else if ((robot.distl.getVoltage() < distance) && (robot.distr.getVoltage() < distance)) {
//                 robot.driveTank(-.15, -.15);
//            }else if ((robot.distl.getVoltage() < distance) && (robot.distr.getVoltage() == distance)) {   //once the distances are equal, the whole robot will move back
//                robot.moveRight(.15);
//            }
//            else if ((robot.distl.getVoltage() == distance) && (robot.distr.getVoltage() < distance)) {   //once the distances are equal, the whole robot will move back
//                robot.moveLeft(.15);
//            }
//        }

    }
}
