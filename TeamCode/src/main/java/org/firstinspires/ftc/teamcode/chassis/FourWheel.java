package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by davis on 9/27/16.
 */
public abstract class FourWheel {
  HardwareMap hwMap;
  DcMotor FL; // v1
  DcMotor FR; // v2
  DcMotor BL; // v3
  DcMotor BR; // v4

  public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;
    FL = hwMap.dcMotor.get("FL");
    FR = hwMap.dcMotor.get("FR");
    BL = hwMap.dcMotor.get("BL");
    BR = hwMap.dcMotor.get("BR");

    FL.setDirection(DcMotor.Direction.REVERSE);
    BL.setDirection(DcMotor.Direction.REVERSE);
  }

  public void moveLeft(double pow) {
    FL.setPower(pow);
    BL.setPower(pow);
  }

  public void moveRight(double pow) {
    FR.setPower(pow);
    BR.setPower(pow);
  }

  public void driveTank(double l, double r) {
    moveLeft(l);
    moveRight(r);
  }

  public void stopMotors() {
    driveTank(0, 0);
  }
}
