package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by davis on 11/26/16.
 */
@TeleOp(name="Tank Drive", group="teleop")
public class AltDrive extends HolonomicDrive{
  void drive(Gamepad gp) {
    tankDrive(gp);
  }
}
