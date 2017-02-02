package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by davis on 11/17/16.
 */
@Disabled
@Autonomous(name="Blue Auto", group="auto")
public class AutoBlue extends AutoV1 {
  double getDir() {
    return 1;
  }

  String getFirstTarget() {
    return "wheels";
  }

  String getSecondTarget() {
    return "legos";
  }
}
