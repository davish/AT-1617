package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by davis on 11/17/16.
 */
@Autonomous(name="Blue Auto", group="auto")
public class AutoBlue extends AutoV2 {
  double getDir() {
    return -1;
  }

  String getFirstTarget() {
    return "wheels";
  }

  String getSecondTarget() {
    return "legos";
  }
}
