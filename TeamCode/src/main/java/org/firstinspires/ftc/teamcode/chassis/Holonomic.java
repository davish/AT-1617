package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by davis on 9/27/16.
 */
public abstract class Holonomic extends FourWheel{

  public abstract void move(double pow, double angle, double rot);

}
