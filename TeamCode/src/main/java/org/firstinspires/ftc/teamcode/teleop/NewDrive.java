package org.firstinspires.ftc.teamcode.teleop;

/**
 * Created by davis on 9/13/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Atlas;
import org.firstinspires.ftc.teamcode.chassis.Orion;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Atlas Drive", group="teleop")
public class NewDrive extends OpMode {

  Atlas robot = new Atlas();

  public void init() {
    robot.init(hardwareMap);
    telemetry.addData(">", "Initialization complete.");
    telemetry.update();
  }

  public void loop() {
    drive(gamepad1);
    pickup(gamepad1, gamepad2);
    launch(gamepad1);
    transfer(gamepad1);
    lift(gamepad1);

    telemetry.update();
  }

  void lift(Gamepad gp) {
    if (gp.left_bumper)
      robot.lift.setPower(1);
    else if (gp.right_bumper)
      robot.lift.setPower(-1);
    else
      robot.lift.setPower(0);
  }

  double chamberPos = .5;
  void transfer (Gamepad gp)
  {
      // Slowly move the chamber using the dpad.
      if (gp.a)
        chamberPos += .01;
      else if (gp.b)
        chamberPos -= .01;
      chamberPos = FtcUtil.scale(chamberPos, 0, 1);
      robot.transervo(chamberPos);
    }

  void pickup(Gamepad gp, Gamepad gp2) {
    if (gp.right_trigger > .1 || gp2.right_trigger > .1)
      robot.runPickup(1);

    else if (gp.left_trigger > .1 || gp2.left_trigger > .1)
      robot.runPickup(-1);
    else
      robot.runPickup(0);
  }

  boolean lastState = false;
  boolean isMoving = false;
  void launch(Gamepad gp) {
    // We want this to be a button tap. If the button is pressed, set isMoving to true, if it's
    // been pressed in the past, persist that state.
    isMoving = (gp.y || isMoving) && !gp.x;
    // if we're moving and we're transitioning from open to closed, stop moving
    if (isMoving && !lastState && robot.catapultLoaded()) {
      isMoving = false;
    }

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
        angle = Math.PI;
      else // go backwards if stick is pushed down
        angle = 0;
    } else { // if sideways movement is the strongest input,
      pow = 1; // 100% power.
      if (strafe > 0) // go right
        angle = -Math.PI/2;
      else // go left
        angle = Math.PI/2;
    }

    // SLOW MODE
    rot = FtcUtil.threshold(rotate, FtcUtil.sign(rotate));

    if (gp.dpad_up) {
      angle = Math.PI;
      pow = .5;
    } else if (gp.dpad_down) {
      angle = 0;
      pow = .5;
    } else if(gp.dpad_left) {
      rot = -.3;
    } else if(gp.dpad_right) {
      rot = .3;
    }
//    rot = FtcUtil.scale(rot, -.3, .3);

    telemetry.addData("Distance", robot.getTicks());
    robot.move(pow, angle, rot);
  }
}