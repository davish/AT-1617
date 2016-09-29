package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FtcUtil;

/**
 * Created by davis on 9/27/16.
 */
public abstract class Holonomic extends FourWheel{

  public abstract void move(double pow, double angle, double rot);

  /**
   * Use PID in order to move directly in the desired angle, without rotation.
   * @param pow Motor power (approximate speed)
   * @param angle Desired angle
   * @param actual Orientation/rotation measurement
   * @param target Desired orientation/rotation
   */
  public void moveStraight(double pow, double angle, double actual, double target) {
    double Kp = -0.007;
    double error = actual - target;

    double rot = FtcUtil.motorScale(error*Kp);
    this.move(pow, angle, rot);
  }

  /**
   * Use PID to move without rotation, when the target angle is 0.
   */
  public void moveStraight(double pow, double angle, double actual) {
    this.moveStraight(pow, angle, actual, 0);
  }

}
