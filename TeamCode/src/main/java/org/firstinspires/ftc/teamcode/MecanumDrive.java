package org.firstinspires.ftc.teamcode;

/**
 * Created by davis on 9/13/16.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDrive extends OpMode {

  HardwareMecanum robot = new HardwareMecanum();

  double lastPow = 0;
  double targetHeading = 0;

  public void init() {
    robot.init(hardwareMap);
  }

  public void loop() {
    robot.imu.update();

    driveRobot(gamepad1);
  }

  /**
   * Mecanum drive.
   *
   * Has three inputs: vx, vy, and r.
   * vx and vy determine the power and direction of motion, r determines the rotation rate.
   * vx and vy should be on the same joystick, r should be the other joystick.
   *
   * @param gp gamepad that controls driving
   *
   *
   */
  void driveRobot(Gamepad gp) {
    double vx = FtcUtil.threshold(gp.left_stick_x);
    double vy = FtcUtil.threshold(gp.left_stick_y);
    double rot = FtcUtil.threshold(gp.right_stick_x);

    double pow = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    double angle = Math.atan2(vy, vx) + Math.PI/2;

    if (angle < 0)
      angle += Math.PI*2;

//    // if we are moving now, and we weren't moving last loop, set a new heading.
//    if (pow > 0 && lastPow == 0) {
//      targetHeading = robot.imu.heading();
//    }
//    // If we're not rotating intentionally, then override the rotation with PID control so we're continually facing straight.
//    if (pow > Math.abs(rot) && pow > 0) {
//      rot = robot.PID(robot.imu.heading(), targetHeading);
//    }

    robot.omniMove(pow, angle, rot);
    lastPow = pow;
  }
}