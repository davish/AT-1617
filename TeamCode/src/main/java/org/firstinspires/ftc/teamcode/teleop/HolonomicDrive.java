package org.firstinspires.ftc.teamcode.teleop;

/**
 * Created by davis on 9/13/16.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Holonomic;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Holonomic Drive", group="TeleOp")
public class HolonomicDrive extends OpMode {

  Holonomic robot = new Mecanum();

  public void init() {
    robot.init(hardwareMap);
    telemetry.addData(">", "Initialization complete.");
    telemetry.update();
  }

  public void loop() {
    drive(gamepad1);
    pickup(gamepad1);
    transfer(gamepad2);
//    launch(gamepad2);
    altLaunch(gamepad1);
    telemetry.update();
  }


  void pickup(Gamepad gp) {
    if (gp.right_trigger > .1)
      robot.runPickup(1);
    else if (gp.left_trigger > .1)
      robot.runPickup(-1);
    else
      robot.runPickup(0);
  }

  double chamberPos = robot.PIVOT_SENSERIGHT;
  void transfer(Gamepad gp) {
    if (gp.dpad_right)
      chamberPos += .01;
    else if (gp.dpad_left)
      chamberPos -= .01;
    chamberPos = FtcUtil.scale(chamberPos, robot.PIVOT_HITRIGHT-.05, robot.PIVOT_SENSERIGHT);
    robot.pivot(chamberPos);
  }

  boolean debounce = false;
  boolean wasDown = false;
  boolean hasLaunched = false;
  void launch(Gamepad gp) {
    boolean shouldMove;
    if (Math.abs(gp.right_trigger) > .1) {
      if (robot.catapultLoaded()) {
        shouldMove = !hasLaunched;
        wasDown = true;
      } else {
        if (!debounce) // If
          wasDown = true;
        shouldMove = true;
        hasLaunched = wasDown;
      }
      debounce = true;
    } else {
      shouldMove = hasLaunched = wasDown = false;
      debounce = false;
    }
    if (shouldMove)
      robot.runChoo(1);
    else
      robot.runChoo(0);
  }

  boolean lastState = false;
  boolean isMoving = false;
  void altLaunch(Gamepad gp) {
    // We want this to be a button tap. If the button is pressed, set isMoving to true, if it's
    // been pressed in the past, persist that state.
    isMoving = (gp.y || isMoving) && !gp.x;
    // if we're moving and we're transitioning from open to closed, stop moving
    if (isMoving && !lastState && robot.catapultLoaded()) isMoving = false;

    if (isMoving)
      robot.runChoo(1);
    else
      robot.runChoo(0);
    lastState = robot.catapultLoaded();
  }

  void drive(Gamepad gp) {
    double rot;
    double angle;
    double pow;

    double forward = gp.right_stick_y;
    double strafe = gp.right_stick_x;
    double rotate = gp.left_stick_x;

    if (FtcUtil.threshold(forward) == 0 && FtcUtil.threshold(strafe) == 0) {
      pow = 0;
      angle = 0;
    } else if (Math.abs(forward) > Math.abs(strafe)) {
      pow = .8;
      if (forward > 0)
        angle = 0;
      else
        angle = Math.PI;
    } else {
      pow = 1;
      if (strafe > 0)
        angle = Math.PI/2;
      else
        angle = -Math.PI/2;
    }

    rot = FtcUtil.threshold(rotate, FtcUtil.sign(rotate));

    robot.move(pow, angle, rot);
  }

  void swivel(Gamepad gp) {
    robot.pivot(gp.left_trigger);
    telemetry.addData("Servo pos", gp.left_trigger);
  }
}