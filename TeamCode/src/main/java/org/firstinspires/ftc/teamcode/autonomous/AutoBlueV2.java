package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 2/3/17.
 */
@Autonomous(name="Blue Auto v2", group="tests")
public class AutoBlueV2 extends AutoBase {
        double power = -.3;
        int color = 0;
        double getDir() {
            return 1;
        }
        public void run() throws InterruptedException {
            driveTicks(-power, 1250); // drive forward to shoot
            sleep(SLEEP_TIME);
//    shootParticles();
            rotateDegs(power, 70); // rotate and move towards beacon
            sleep(SLEEP_TIME);
            driveTicks(power, 2200);
            sleep(SLEEP_TIME);
            rotateDegs(power, 45); // rotate into alignment with wall
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
                robot.move(power/2, Math.PI, 0);
            }
            robot.stopMotors();
            sleep(SLEEP_TIME*3);
            // drive forward to align with beacon, then push the proper button
            driveTicks(power/2, 150);
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
                driveTicks(power, 1200); // go with encoders fast until we're close to the line, then
                robot.imu.update();
                h = robot.imu.heading();
                while (robot.getDistanceAway() >15 && opModeIsActive()) {
                    robot.imu.update();
                    robot.moveStraight(.8, Math.PI/2, robot.imu.heading(), h);
                }

                while (!robot.isOnLinel() && opModeIsActive()) {
                    robot.move(power / 2, 0, 0);
                }
                robot.stopMotors();
                sleep(SLEEP_TIME);
                driveTicks(power/2, 150);
                sleep(SLEEP_TIME);
                pushButton((int) getDir());

                //cap ball
                if(settings.knockCapBall)
                {
                    rotateDegs(power, 150);
                    driveTicks(power, 2500); //figure out how many ticks
                }

            }
            else if (settings.knockCapBall){
                rotateDegs(power, 90);
                driveTicks(power, 2000);
                rotateDegs(power, 85);
            }


        }

    }
