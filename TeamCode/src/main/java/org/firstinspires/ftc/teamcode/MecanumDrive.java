package org.firstinspires.ftc.teamcode;

/**
 * Created by davis on 9/13/16.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecanumDrive extends OpMode {

  HardwareMecanum robot = new HardwareMecanum();

  public void init() {
    robot.init(hardwareMap);
  }

  public void loop() {
    driveRobot(gamepad1);
  }

  /**
   * Mecanum drive.
   *
   * Has three inputs: vx, vy, and r.
   * vx and vy determine the power and direction of motion, r determines the rotation rate.
   * vx and vy should be on the same joystick, r should be the other joystick.
   *
   * @param gp gamepad
   *
   *
   */
  void driveRobot(Gamepad gp) {
    double vx = FtcUtil.threshold(gp.left_stick_x);
    double vy = FtcUtil.threshold(gp.left_stick_y);
    double r = FtcUtil.threshold(gp.right_stick_x);

    double pow = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    double angle = Math.atan2(vy, vx) + Math.PI/2;
    double rot = Math.abs(r) > .1 ? r : 0;
    if (angle < 0)
      angle += Math.PI*2;

    drive(pow, angle, rot);
  }


  /**
   * Drive in a certain direction with a mecanum chassis
   * @param pow Motor power
   * @param angle Angle to drive towards
   * @param rot speed of rotation
   */
  void drive(double pow, double angle, double rot) {
    pow = FtcUtil.motorScale(pow);
    rot = FtcUtil.motorScale(rot);

    // Adding PI/4 ensures that 0 degrees is straight ahead
    double vx = pow*Math.cos(angle+Math.PI/4);
    double vy = pow*Math.sin(angle+Math.PI/4);

    double[] V = {vx+rot, vy-rot, vy+rot, vx-rot}; // contains motor powers for each motor.

    /*
     * because of adding/subtracting rotation, these numbers could be between [-2,2].
     * To get around this, find the maximum motor power, and divide all of them by that
     * so that the proportions stay the same but now it's between [-1,1].
     */

    // find max
    double m = 0.0;
    for (double v : V)
      if (Math.abs(v) > m)
        m = v;

    double mult = Math.max(Math.abs(pow), Math.abs(rot));
    // adjust values, still keeping power in mind.
    if (m != 0) // if the max power isn't 0 (can't divide by 0)
      for(int i = 0; i < V.length; i++)
        V[i] = Math.abs(mult) * (V[i]/Math.abs(m));

    // finally, set motor powers.
    robot.FL.setPower(V[0]);
    robot.FR.setPower(V[1]);
    robot.BL.setPower(V[2]);
    robot.BR.setPower(V[3]);

  }
}