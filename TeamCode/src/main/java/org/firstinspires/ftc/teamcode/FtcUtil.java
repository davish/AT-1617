package org.firstinspires.ftc.teamcode;

/**
 * Created by davis on 5/22/16.
 */
public class FtcUtil {
  public static final double JOYSTICK_THRESHOLD = .1;

  public static double servoScale(double d) {
    if (d < 0.0) return 0.0;
    if (d > 1.0) return 1.0;
    return d;
  }

  public static double motorScale(double d) {
    if (d < -1.0) return -1.0;
    if (d > 1.0)  return 1.0;
    return d;
  }

  public static double sign(double d) {
    if (d >= 0.0) return 1.0;
    else return -1.0;
  }
  public static double threshold(double d) {
    if (Math.abs(d) > JOYSTICK_THRESHOLD)
      return d;
    else
      return 0.0;
  }
}
