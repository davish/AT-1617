package org.firstinspires.ftc.teamcode.teleop;

/**
 * Created by davis on 9/13/16.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.chassis.Atlas;

/**
 * Created by davis on 5/22/16.
 */
@TeleOp(name="Atlas Drive", group="teleop")
public class NewDrive extends OpMode {

  Atlas robot = new Atlas();

  public void init() {
    robot.init(hardwareMap, false);
    telemetry.addData(">", "Initialization complete.");
    telemetry.update();
  }

  public void loop() {
    drive(gamepad1);
    pickup(gamepad2);
    launch(gamepad1, gamepad2);

    transfer(gamepad2);
    lift(gamepad2);
//    boop(gamepad2);



    telemetry.addData("servo", chamberPos);
    telemetry.update();
  }

  void boop(Gamepad gp) {
    boolean go_out = gp.right_bumper;
    boolean go_in = gp.left_bumper;

    if (go_out) {
      robot.pushOut();
    }
    else if (go_in) {
      robot.pushIn();
    }
    else {
      robot.pushStop();
    }
  }

  void lift(Gamepad gp) {
    boolean go_up = gp.left_stick_y > .1 && gp.right_stick_y > .1;
    boolean go_down = gp.left_stick_y < -.1 && gp.right_stick_y < -.1;

    telemetry.addData("left stick", gp.left_stick_y);
    telemetry.addData("right stick", gp.right_stick_y);

    if (go_up)
      robot.lift.setPower(1);
    else if (go_down)
      robot.lift.setPower(-1);
    else
      robot.lift.setPower(0);
  }

  void transfer(Gamepad gp) {
    if (!gp.x)
      transferMain(gp);
    else
      transferOverride(gp);
  }

  void launch(Gamepad gp, Gamepad gp2) {
    if (!gp2.a)
      launchMain(gp);
    else {
      launchOverride(gp2);
    }
  }

  double chamberPos = robot.REST_POSITION;
  void transferOverride(Gamepad gp) {
    // Slowly move the chamber using the dpad.
    if (gp.dpad_down)
      chamberPos += robot.STEP_SIZE;
    else if (gp.dpad_up)
      chamberPos -= robot.STEP_SIZE;
    chamberPos = FtcUtil.scale(chamberPos, 0, 1);
    robot.transervo(chamberPos);
  }

  int transferState = 0;
  long startWait;
  void transferMain(Gamepad gp) {
    switch(transferState) {
      case 0:
        if (gp.b)
          transferState = 1;
        break;
      case 1:
        // might have to switch to (chamberPos < LOAD_POSITION)
        if (chamberPos > robot.LOAD_POSITION) { // as long as the servo isn't at the limit,
          chamberPos -= robot.STEP_SIZE; // keep the transfer going.
        } else {
          transferState = 2;
          startWait = System.currentTimeMillis();
        }
        break;
      case 2:
        if (System.currentTimeMillis() - startWait > robot.DELAY_TIME)
          transferState = 3;
        break;
      case 3:
        // might have to switch to (chamberPos > REST_POSITION
        if (chamberPos < robot.REST_POSITION)
          chamberPos += robot.STEP_SIZE;
        else
          transferState = 0;
        break;
      default:
        transferState = 0;
    }

    robot.transervo(chamberPos);
  }

  void pickup(Gamepad gp) {
    if (gp.right_trigger > .1)
      robot.runPickup(1);

    else if (gp.left_trigger > .1)
      robot.runPickup(-1);
    else
      robot.runPickup(0);
  }

  boolean lastState = false;
  boolean isMoving = false;
  void launchMain(Gamepad gp) {
    // We want this to be a button tap. If the button is pressed, set isMoving to true, if it's
    // been pressed in the past, persist that state.
    isMoving = (gp.y || isMoving) && !gp.x;
    // if we're moving and we're transitioning from open to closed, stop moving
    if (isMoving && !lastState && robot.catapultLoaded()) {
      isMoving = false;
      transferState = 1;
    }

    if (isMoving)
      robot.runChoo(1);
    else
      robot.runChoo(0);
    lastState = robot.catapultLoaded();
  }

  void launchOverride(Gamepad gp) {
    double throttle = gp.left_stick_x*.5;
    isMoving = false;
    robot.runChoo(Math.abs(FtcUtil.threshold(throttle)));
  }

  double mult = 1;
  long timeSince = -1;
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
      pow = 1; // power is 80% forward
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
      rot = -.5;
    } else if(gp.dpad_right) {
      rot = .5;
    }
//    rot = FtcUtil.scale(rot, -.3, .3);

    if (gp.right_trigger > .5 && pow > 0) {
      if (timeSince == -1)
        timeSince = System.currentTimeMillis();
      mult = Math.max(mult - .2 * (System.currentTimeMillis() - timeSince)/1000, 0);
    } else {
      mult = 1;
      timeSince = -1;
    }
//    telemetry.addData("mult", mult);
    telemetry.addData("Distance", robot.getTicks());
    robot.move(pow*mult, angle, rot);
  }
}