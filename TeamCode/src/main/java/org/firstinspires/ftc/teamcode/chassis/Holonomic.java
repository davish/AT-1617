package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FtcUtil;

/**
 * Created by davis on 9/27/16.
 */
public abstract class Holonomic extends FourWheel{

  double integral = 0.0; // Accumulation of error over time. Used in PID controller.
  double lastError = 0.0;

  /*
   PID constants for the chassis. These are abstract methods since each chassis (Mecanum, Omni, etc)
   will have different values for each.
   */
  abstract double getKp();
  abstract double getKi();
  abstract double getKd();

  abstract double getPhoneOffset();

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

    double change = (error - lastError);
    lastError = error;

    double PID = getKp()*error + getKi()*integral + getKd()*change;

    double rot = FtcUtil.motorScale(PID);
    this.move(pow, angle, rot);
  }

  /**
   * Use PID to move without rotation, when the target angle is 0.
   */
  public void moveStraight(double pow, double angle, double actual) {
    this.moveStraight(pow, angle, actual, 0);
  }

  public void alignWithTarget(float[] pos, float heading, double SPEED) {
    double pow = 0, angle = 0;

    if (Math.abs(pos[1] - getPhoneOffset()) > 50) {
      pow = SPEED;
      angle = FtcUtil.sign(pos[1]) * -Math.PI/2;
    }
    else {
      angle = 0;
      pow = SPEED;
    }

    this.moveStraight(pow, angle, heading);
  }

}
