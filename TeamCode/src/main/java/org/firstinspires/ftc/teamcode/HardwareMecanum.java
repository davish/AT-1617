package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by davis on 9/13/16.
 */
public class HardwareMecanum {
  HardwareMap hwMap;


  /**
   *          Front
   *      1 |-------| 2
   * Left    -------     Right
   *         -------
   *      3 |-------| 4
   *           Back
   *
   *
   */


  DcMotor FL; // v1
  DcMotor FR; // v2
  DcMotor BL; // v3
  DcMotor BR; // v4

  public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;
    FR = hwMap.dcMotor.get("front right");
    BR = hwMap.dcMotor.get("back right");
    FL = hwMap.dcMotor.get("front left");
    BL = hwMap.dcMotor.get("back left");

    FR.setDirection(DcMotor.Direction.REVERSE);
    BR.setDirection(DcMotor.Direction.REVERSE);
  }

  void moveLeft(double pow) {
    FL.setPower(pow);
    BL.setPower(pow);
  }

  void moveRight(double pow) {
    FR.setPower(pow);
    BR.setPower(pow);
  }

  void drive(double l, double r) {
    moveLeft(l);
    moveRight(r);
  }

  void stopMotors() {
    drive(0, 0);
  }
}
