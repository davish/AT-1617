package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by davis on 9/21/16.
 */
public class IMU {
  BNO055IMU imu;
  Orientation angles;

  double forwardHeading = 0.0;

  public IMU(BNO055IMU imu) {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    imu.initialize(parameters);
    this.imu = imu;
  }

  public void update() {
    angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
  }

  public double heading() {
    if (angles != null) {
      double h = angles.firstAngle + forwardHeading;
      if (h < 0) h += 360;
      return h;
    }
    else
      return 0.0;
  }

  public double roll() {
    if (angles != null)
      return angles.secondAngle;
    else
      return 0.0;
  }

  public double pitch() {
    if (angles != null)
      return angles.thirdAngle;
    else
      return 0.0;
  }
}
