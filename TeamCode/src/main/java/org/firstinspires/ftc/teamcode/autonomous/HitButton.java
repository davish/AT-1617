package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.Orion;

/**
 * Created by davis on 10/20/16.
 */
@Autonomous(name="Hit Button", group="test")
public class HitButton extends AutoBase {
  double getDir() {return 0;}
  public void run() throws InterruptedException {
    approachBeacon();
    sleep(SLEEP_TIME);
    pushButton((int)getDir()); // code to push beacon
  }
}
