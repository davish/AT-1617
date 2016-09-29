package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FtcUtil;

/**
 * Created by davis on 9/27/16.
 */
public abstract class Holonomic extends FourWheel{

  double integral = 0.0; // Accumulation of error over time. Used in PID controller.

  final double Kp = -0.007; // Proportional constant for PID. Seems to work pretty well at -.007.
  // Integral constant for PID. Set for 0 so it doesn't affect anything now, but play with this for
  // the best result.
  final double Ki = 0;

  public abstract void move(double pow, double angle, double rot);

  /**
   * Use PID in order to move directly in the desired angle, without rotation.
   * @param pow Motor power (approximate speed)
   * @param angle Desired angle
   * @param actual Orientation/rotation measurement
   * @param target Desired orientation/rotation
   */
  public void moveStraight(double pow, double angle, double actual, double target) {
    double error = actual - target;
    integral += error;

    double PID = Kp*error + Ki+integral;

    double rot = FtcUtil.motorScale(PID);
    this.move(pow, angle, rot);
  }

  /**
   * Use PID to move without rotation, when the target angle is 0.
   */
  public void moveStraight(double pow, double angle, double actual) {
    this.moveStraight(pow, angle, actual, 0);
  }

}
