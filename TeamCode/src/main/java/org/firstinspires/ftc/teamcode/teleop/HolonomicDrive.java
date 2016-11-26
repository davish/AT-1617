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
    launch(gamepad1);
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
    // Slowly move the chamber using the dpad.
    if (gp.dpad_right)
      chamberPos += .01;
    else if (gp.dpad_left)
      chamberPos -= .01;
    chamberPos = FtcUtil.scale(chamberPos, robot.PIVOT_HITRIGHT, robot.PIVOT_SENSERIGHT);
    robot.pivot(chamberPos);
  }

  boolean lastState = false;
  boolean isMoving = false;
  void launch(Gamepad gp) {
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

    // Standard FPS xbox controls: Left stick for movement, right stick for rotation/orientation.
    double forward = gp.left_stick_y;
    double strafe = gp.left_stick_x;
    double rotate = gp.right_stick_x;
    // *** Put forward and strafe on right stick and rotate on left stick for Ryan's alternate drivemode.

    if (FtcUtil.threshold(forward) == 0 && FtcUtil.threshold(strafe) == 0) { // if no joystick is engaged,
      pow = 0; // power is 0.
      angle = 0;
    } else if (Math.abs(forward) > Math.abs(strafe)) { // If forward is greater than sideways,
      pow = .8; // power is 80% forward
      if (forward > 0) // Go forwards if stick is pushed up
        angle = 0;
      else // go backwards if stick is pushed down
        angle = Math.PI;
    } else { // if sideways movement is the strongest input,
      pow = 1; // 100% power.
      if (strafe > 0) // go right
        angle = Math.PI/2;
      else // go left
        angle = -Math.PI/2;
    }

    // SLOW MODE
    if (gp.dpad_up) {
      angle = Math.PI;
      pow = .5;
    } else if (gp.dpad_down) {
      angle = 0;
      pow = .5;
    }

    rot = FtcUtil.threshold(rotate, FtcUtil.sign(rotate));

    robot.move(pow, angle, rot);
  }

  void lights(Gamepad gp) {
    if (gp.dpad_up) {
      robot.blueLights.setState(true);
      robot.redLights.setState(false);
      robot.greenLights.setState(false);
    }
    else if (gp.dpad_down) {
      robot.blueLights.setState(false);
      robot.redLights.setState(false);
      robot.greenLights.setState(true);
    }
  }

  void swivel(Gamepad gp) {
    robot.pivot(gp.left_trigger);
    telemetry.addData("Servo pos", gp.left_trigger);
  }
}