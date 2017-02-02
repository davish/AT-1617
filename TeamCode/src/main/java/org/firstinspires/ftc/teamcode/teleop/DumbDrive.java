package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by davis on 1/30/17.
 */
@Disabled
@TeleOp(name="drive")
public class DumbDrive extends OpMode{

  DcMotor FL;
  DcMotor BL;
  DcMotor FR;
  DcMotor BR;
  public void init() {
    FL = hardwareMap.dcMotor.get("FL");
    BL = hardwareMap.dcMotor.get("BL");
    FR = hardwareMap.dcMotor.get("FR");
    BR = hardwareMap.dcMotor.get("BR");

    FL.setDirection(DcMotorSimple.Direction.REVERSE);
    BL.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  void drive(float fl, float bl, float fr, float br) {
    FL.setPower(fl);
    BL.setPower(bl);
    FR.setPower(fr);
    BR.setPower(br);

  }

  public void loop() {
    if (gamepad1.dpad_up) {
      drive(1,1,1,1);
    }
    else if (gamepad1.dpad_down) {
      drive(-1, -1, -1, -1);
    }
    else if (gamepad1.dpad_right) {
      drive(1,-1,-1,1);
    }
    else if (gamepad1.dpad_left) {
      drive(-1,1,1,-1);
    }
    else if (gamepad1.b) {
      drive(1,1,-1,-1);
    }
    else if (gamepad1.x) {
      drive(-1,-1,1,1);
    }
    else if (gamepad1.a) {
      FR.setPower(1);
      BR.setPower(1);
    }
    else if (gamepad1.y) {
      FR.setPower(-1);
      BR.setPower(-1);
    }
    else if (gamepad1.right_bumper) {
      FL.setPower(1);
    }
    else if (gamepad1.left_bumper) {
      FL.setPower(-1);
    }
    else if (gamepad1.right_trigger > .1) {
      BL.setPower(1);
    }
    else if (gamepad1.left_trigger > .1) {
      BL.setPower(-1);
    }
    else {
      drive(0,0,0,0);
    }



  }

}
