package org.firstinspires.ftc.teamcode.teleop;

/**
 * Created by davis on 9/13/16.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.chassis.Omni;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Holonomic Drive", group="TeleOp")
public class HolonomicDrive extends OpMode {

  Holonomic robot = new Mecanum();

  public void init() {
    robot.init(hardwareMap);
  }

  public void loop() {
//    driveRobot(gamepad1);
    dpadDrive(gamepad1);
    swivel(gamepad1);
    telemetry.addData("Line Sensor output", robot.lineSensor.getVoltage());
    telemetry.update();
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

    telemetry.addData("angle", Math.toDegrees(angle));
    robot.move(pow, angle, rot);
  }

  void dpadDrive(Gamepad gp) {
    double rot = FtcUtil.threshold(gp.right_stick_x);
    double angle;
    double pow = .8;
    if (gp.dpad_up)
      angle = 0;
    else if (gp.dpad_right)
      angle = Math.PI/2;
    else if (gp.dpad_down)
      angle = Math.PI;
    else if (gp.dpad_left)
      angle = -Math.PI/2;
    else {
      angle = 0;
      pow = 0;
    }
    robot.move(pow, angle, rot);
  }

  void swivel(Gamepad gp) {
    robot.pivot(gp.left_trigger);
    telemetry.addData("Servo pos", gp.left_trigger);
  }
}