package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 2/3/17.
 */
@Autonomous(name="Blue Auto v2", group="tests")
public class AutoBlueV2 extends AutoBase {
        int color = 0;
        double getDir() {
            return 1;
        }
        public void run() throws InterruptedException {
            driveTicks(-SPEED, 1250); // drive forward to shoot
            sleep(SLEEP_TIME);
//    shootParticles();
            rotateDegs(SPEED, 70); // rotate and move towards beacon
            sleep(SLEEP_TIME);
            driveTicks(SPEED, 2200);
            sleep(SLEEP_TIME);
            rotateDegs(SPEED, 45); // rotate into alignment with wall
            sleep(SLEEP_TIME);

            robot.imu.update(); // strafe until we're within pushing range
            double h = robot.imu.heading();
            while (robot.getDistanceAway() > 15 && opModeIsActive()) {
                robot.imu.update();
                robot.moveStraight(.8, Math.PI/2, robot.imu.heading(), h);
            }
            robot.stopMotors();
            sleep(SLEEP_TIME*2);
            // move backwards until we're aligned with the line.
            while (!robot.isOnLinel() && opModeIsActive()) {
                robot.move(SPEED /2, Math.PI, 0);
            }
            robot.stopMotors();
            sleep(SLEEP_TIME*3);
            // drive forward to align with beacon, then push the proper button
            driveTicks(SPEED /2, 150);
            pushButton((int)getDir()); // code to push beacon
            sleep(SLEEP_TIME);

            if (settings.beacon2) {
               robot.imu.update();
                 h = robot.imu.heading();
                while (robot.getDistanceAway() < 20 && opModeIsActive()) {
                    robot.imu.update();
                    robot.moveStraight(.8, Math.PI/2, robot.imu.heading(), h);
                }
                robot.stopMotors();
                driveTicks(SPEED, 1200); // go with encoders fast until we're close to the line, then
                robot.imu.update();
                h = robot.imu.heading();
                while (robot.getDistanceAway() >15 && opModeIsActive()) {
                    robot.imu.update();
                    robot.moveStraight(.8, Math.PI/2, robot.imu.heading(), h);
                }

                while (!robot.isOnLinel() && opModeIsActive()) {
                    robot.move(SPEED / 2, 0, 0);
                }
                robot.stopMotors();
                sleep(SLEEP_TIME);
                driveTicks(SPEED /2, 150);
                sleep(SLEEP_TIME);
                pushButton((int) getDir());

                //cap ball
                if(settings.knockCapBall)
                {
                    rotateDegs(SPEED, 150);
                    driveTicks(SPEED, 2500); //figure out how many ticks
                }

            }
            else if (settings.knockCapBall){
                rotateDegs(SPEED, 90);
                driveTicks(SPEED, 2000);
                rotateDegs(SPEED, 85);
            }


        }

    }
