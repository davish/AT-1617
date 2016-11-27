package org.firstinspires.ftc.teamcode.autonomous;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.chassis.Mecanum;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;

import java.io.File;
import java.io.IOException;

/**
 * Created by davis on 11/25/16.
 */
public abstract class AutoBase extends LinearOpMode {
  Mecanum robot;
  void setup() throws InterruptedException{
    Gson gson = new Gson();
    robot = new Mecanum();
    File sfile = AppUtil.getInstance().getSettingsFile("auto_settings.json");
    Settings settings;
    try {
      settings = gson.fromJson(ReadWriteFile.readFileOrThrow(sfile), Settings.class);
    } catch (IOException e) {
      settings = new Settings();
    }
    robot.init(hardwareMap);
    Vuforia vuforia = new Vuforia();
    telemetry.addData(">", "Vuforia initialized.");
    telemetry.update();
    waitForStart();
    vuforia.activate();
    telemetry.addData(">", settings.delay + " second delay");
    telemetry.update();
    sleep(settings.delay*1000);
  }

}
