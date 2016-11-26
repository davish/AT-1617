package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

/**
 * Created by davis on 10/6/16.
 */
public abstract class AutoV1 extends LinearOpMode {
  Holonomic robot;

  double SPEED = 0.6;
  double FAST_SPEED = 1.0;

  String FIRST_TARGET = getFirstTarget();
  String SECOND_TARGET = getSecondTarget();

  abstract String getFirstTarget();
  abstract String getSecondTarget();
  abstract double getDir();

  public void runOpMode() throws InterruptedException {
    robot = new Mecanum();
    robot.init(hardwareMap);

    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();

    // Move forward for .4 seconds
    robot.move(SPEED, 0, 0);
    sleep(400);
    robot.stopMotors();
    sleep(500);

    // turn 55 degrees
    do {
      robot.imu.update();
      robot.move(0, 0, FAST_SPEED*getDir());
    } while (Math.abs(robot.imu.heading()) < 55 && opModeIsActive());
    robot.stopMotors();
    sleep(500);

    // Move forward to get in range of vision targets
    double target = robot.imu.heading();
    long startTime = System.currentTimeMillis();
    do {
      long currentTime = System.currentTimeMillis();
      // if we haven't seen the target within 6 seconds, stop moving.
      if (currentTime - startTime > 6000) {
        while (opModeIsActive())
          robot.stopMotors();
      }
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    } while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive());
    sleep(500);
    // Keep moving until we're 70cm away from the first target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 700 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(FAST_SPEED, 0, robot.imu.heading(), target);
      idle();
    }
    robot.stopMotors();
    sleep(500);

    // Turn until we're 90 degrees away from how we started.
    do {
      robot.imu.update();
      robot.move(0, 0, FAST_SPEED*getDir());
    } while (Math.abs(robot.imu.heading()) < (getDir() == 1 ? 86 : 75) && opModeIsActive());
    robot.stopMotors();

    sleep(500);
    // Get current heading for PID strafing
    robot.imu.update();
    double h = robot.imu.heading();

    // if we can't see the vision target, strafe until we can.
    while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive())
      robot.move(1.0, Math.PI / 2 * getDir(), 0);
    robot.stopMotors();
    sleep(500);

    // move forward until we're 36 cm away from the vision target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 360 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, 0, robot.imu.heading(), h);
      idle();
    }
    robot.stopMotors();
    sleep(500);

    // get our position relative to the vision target (are we on left/right side)
    double dir = FtcUtil.sign(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]);
    // Strafe sideways until we're lined up with the vision target
    while (vuforia.getAlignment(FIRST_TARGET) == null || Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[1]-(getDir() == 1 ? 40 : -100)) > 50 && robot.ods.getLightDetected() < .5   && opModeIsActive()) {
      robot.imu.update();
      robot.move(1.0, Math.PI / 2 * dir, 0);
    }
    robot.stopMotors();
    sleep(500);

    // Do another correction to make sure we're aligned with wall after strafe

//    robot.imu.update();
//    if (Math.abs(Math.abs(robot.imu.heading()) - (getDir() == 1 ? 86 : 80)) > 5) {
//      do {
//        robot.imu.update();
//        robot.move(0, 0, FAST_SPEED * getDir());
//      } while (Math.abs(robot.imu.heading()) > (getDir() == 1 ? 86 : 80) && opModeIsActive());
//    }
    sleep(500);

    // Make sure we can see the vision guide before we start moving
    while (vuforia.getAlignment(FIRST_TARGET) == null && opModeIsActive())
      ;
    // move until we're 22.5cm away from the target
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 225 && opModeIsActive()) {
      robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    // Check both sides of beacon, put the right paddle forward.
    int a = robot.hitBeacon(-1*(int)getDir());

    sleep(1000);
    // Go forwards and hit the beacon twice to make sure the button is pressed
    int count = 0;
    if (a != 0) {
      while (opModeIsActive() && count < 2) {
        robot.move(1 * SPEED / 3, 0, 0);
        sleep(700);
        robot.stopMotors();
        sleep(250);
        robot.move(1 * SPEED / 3, Math.PI, 0);
        sleep(700);
        robot.stopMotors();
        sleep(250);
        count++;
      }
    }
    sleep(500);
    robot.imu.update();
    h = robot.imu.heading();

    // Move backwards to align for shooting particle
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) < 520 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, Math.PI, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
      idle();
    }
    robot.stopMotors();
    robot.centerServo();
    sleep(500);

    // Load catapult, and shoot
    while (!robot.catapultLoaded())
      robot.runChoo(1);
    while (robot.catapultLoaded())
      robot.runChoo(1);
    robot.runChoo(0);

    sleep(500);

    // Move back towards vision target until we're 30cm away
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(FIRST_TARGET))[2]) > 300 && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(2 * SPEED / 3, 0, Vuforia.getHeading(vuforia.getAlignment(FIRST_TARGET)));
      idle();
    }
    robot.stopMotors();
    sleep(500);

    // Strafe until we've hit the white line for the second beacon.
    while (!robot.isOnLine() && opModeIsActive()) {
      robot.imu.update();
      robot.moveStraight(1.0, Math.PI/2* dir, robot.imu.heading(), h);
    }
    robot.stopMotors();
    sleep(500);

    // Make sure we can see the second vision guide
    while (vuforia.getAlignment(SECOND_TARGET) == null && opModeIsActive())
      ;

    // Align with vision target and move until we're 22.5cm away
    while (Math.abs(Vuforia.getPosition(vuforia.getAlignment(SECOND_TARGET))[2]) > 225 && opModeIsActive()) {
      robot.moveStraight(SPEED, 0, Vuforia.getHeading(vuforia.getAlignment(SECOND_TARGET)));
    }
    robot.stopMotors();
    sleep(500);
    // Sense second beacon color, put right paddle forward.
    a = robot.hitBeacon(-1*(int)getDir());
    sleep(1000);
    count = 0;
    // Hit beacon.
    if (a != 0) {
      while (opModeIsActive() && count < 2) {
        robot.move(1 * SPEED / 3, 0, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        robot.move(1 * SPEED / 3, Math.PI, 0);
        sleep(500);
        robot.stopMotors();
        sleep(250);
        count++;
      }
    }

  }

  void transferBall() throws InterruptedException{
    double chamberPos = robot.PIVOT_SENSERIGHT;
    while (opModeIsActive() && chamberPos > robot.PIVOT_HITRIGHT) {
      chamberPos -= .01;
      robot.pivot(chamberPos);
      idle();
    }
    sleep(150);
    while (opModeIsActive() && chamberPos < robot.PIVOT_SENSERIGHT) {
      chamberPos += .01;
      robot.pivot(chamberPos);
      idle();
    }
  }
}