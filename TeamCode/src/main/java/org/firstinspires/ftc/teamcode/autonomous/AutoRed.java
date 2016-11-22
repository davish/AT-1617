package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by davis on 11/17/16.
 */

@Autonomous(name="Red Auto", group="auto")
public class AutoRed extends SuperNewAuto{
  double getDir() {
    return -1.0;
  }

  String getFirstTarget() {
    return "gears";
  }

  String getSecondTarget() {
    return "tools";
  }
}


