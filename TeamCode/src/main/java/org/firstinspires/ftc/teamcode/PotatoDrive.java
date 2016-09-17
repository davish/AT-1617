package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by davis on 9/15/16.
 */

@TeleOp(name="Potato Drive", group="TeleOp")
public class PotatoDrive extends OpMode {

  DcMotor FL, FR, BL, BR;

  public void init() {
    FL = hardwareMap.dcMotor.get("FL");
    FR = hardwareMap.dcMotor.get("FR");
    BL = hardwareMap.dcMotor.get("BL");
    BR = hardwareMap.dcMotor.get("BR");

    FL.setDirection(DcMotorSimple.Direction.REVERSE);
    BL.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void loop() {

  }

  void drive(double left, double right) {
    FL.setPower(left);
    BL.setPower(left);
    FR.setPower(right);

  }
}
