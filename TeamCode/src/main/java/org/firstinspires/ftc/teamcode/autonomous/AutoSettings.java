package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;
import java.io.IOException;

/**
 * Created by davis on 11/25/16.
 */
@Autonomous(name="Settings", group="Settings")
public class AutoSettings extends LinearOpMode {
  static final int DEBOUNCE_DELAY = 200;

  public void runOpMode() throws InterruptedException {
    Gson gson = new Gson();
    File sfile = AppUtil.getInstance().getSettingsFile("auto_settings.json");
    Settings settings;
    try {
      settings = gson.fromJson(ReadWriteFile.readFileOrThrow(sfile), Settings.class);
    } catch (IOException e) {
      settings = new Settings();
    }
    telemetry.addData("Score Potential", scorePotential(settings));
    telemetry.addData("Delay (seconds)", settings.delay);
    telemetry.addData("Hit beacon 1", settings.beacon1);
    telemetry.addData("Hit beacon 2", settings.beacon2);
    telemetry.addData("Shoot how many particles", settings.numShots);
    telemetry.addData("Knock off cap ball", settings.knockCapBall);
    telemetry.addData("End on center", settings.endOnCenter);
    telemetry.update();
    waitForStart();

    int menu = 0;
    while (opModeIsActive()) {
      telemetry.addData("Score Potential", scorePotential(settings));
      switch (menu) {
        case 0:
          telemetry.addData(">", "Starting delay");
          telemetry.addData("delay", settings.delay);
          telemetry.addData("a)", "continue");
          if (gamepad1.dpad_up || gamepad1.dpad_down) {
            settings.delay += gamepad1.dpad_up ? 1 : -1;
            settings.delay = Math.max(settings.delay, 0); // can't have negative delay time
            sleep(DEBOUNCE_DELAY);
          }
          if (gamepad1.a) {
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        case 1:
          telemetry.addData(">", "Hit Beacon 1?");
          telemetry.addData("a)", "yes");
          telemetry.addData("b)", "no");
          if (gamepad1.a || gamepad1.b) {
            settings.beacon1 = gamepad1.a;
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        case 2:
          telemetry.addData(">", "Hit Beacon 2?");
          telemetry.addData("a)", "yes");
          telemetry.addData("b)", "no");
          if (gamepad1.a || gamepad1.b) {
            settings.beacon2 = gamepad1.a;
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        case 3:
          telemetry.addData(">", "Shoot how many balls?");
          telemetry.addData("a)", "2");
          telemetry.addData("b)", "1");
          telemetry.addData("x)", "0");
          if (gamepad1.a || gamepad1.b || gamepad1.x) {
            settings.numShots = gamepad1.a ? 2 : gamepad1.b ? 1 : 0;
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        case 4:
          telemetry.addData(">", "Knock off Cap Ball?");
          telemetry.addData("a)", "yes");
          telemetry.addData("b)", "no");
          if (gamepad1.a || gamepad1.b) {
            settings.knockCapBall = gamepad1.a;
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        case 5:
          telemetry.addData(">", "End on center?");
          telemetry.addData("a)", "yes");
          telemetry.addData("b)", "no");
          if (gamepad1.a || gamepad1.b) {
            settings.endOnCenter = gamepad1.a;
            ReadWriteFile.writeFile(sfile, gson.toJson(settings));
            sleep(DEBOUNCE_DELAY);
            menu++;
          }
          break;
        default:
          menu = 0;
          break;
      }
      telemetry.update();
      idle();
    }
  }

  int scorePotential(Settings s) {
    int score = 0;
    if (s.beacon1) score += 30;
    if (s.beacon2) score += 30;
    score += s.numShots * 15;
    if (s.knockCapBall) score += 10;
    if (s.endOnCenter) score += 5;
    return score;
  }
}
