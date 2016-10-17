package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.sensors.IMU;

/**
 * Created by davis on 9/13/16.
 */
public class Omni extends Holonomic{
  /**
   *          Front
   *      1 /-------\ 2
   * Left    -------     Right
   *         -------
   *      3 \-------/ 4
   *           Back
   */

  double getKp() {
    return -0.007;
  }

  double getKi() {
    return 0.0;
  }

  double getKd() {
    return 0.0;
  }

  /**
   * Drive in a certain direction with a mecanum chassis
   * @param pow Base power (magnitude)
   * @param angle Angle to drive towards
   * @param rot speed of rotation
   */
  public void move(double pow, double angle, double rot) {
    pow *= .25;
    rot *= .25;
    pow = FtcUtil.motorScale(pow);
    rot = FtcUtil.motorScale(rot);
    angle += Math.PI/2;
    // Adding PI/4 ensures that 0 degrees is straight ahead
    double vx = -pow*Math.cos(angle+Math.PI/4);
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

    double mult = Math.max(Math.abs(pow), Math.abs(rot)); // If we're just rotating, pow will be 0
    // adjust values, still keeping power in mind.
    if (m != 0) // if the max power isn't 0 (can't divide by 0)
      for(int i = 0; i < V.length; i++)
        V[i] = Math.abs(mult) * (V[i]/Math.abs(m));

    // finally, set motor powers.
    FL.setPower(V[0]);
    FR.setPower(V[1]);
    BL.setPower(V[2]);
    BR.setPower(V[3]);

  }

  double getPhoneOffset() {
    return 0.0;
  }

}
