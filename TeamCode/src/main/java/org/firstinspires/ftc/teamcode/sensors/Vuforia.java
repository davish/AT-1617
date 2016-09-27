package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class Vuforia {
  VuforiaLocalizer vuforia;

  float mmPerInch        = 25.4f;
  float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
  float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels


  VuforiaTrackable red1;
  VuforiaTrackable red2;
  VuforiaTrackable blue1;
  VuforiaTrackable blue2;
  VuforiaTrackables visionGuides;
  List<VuforiaTrackable> allTrackables;


  public Vuforia() {
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    parameters.vuforiaLicenseKey = "Aah86E7/////AAAAGbepZIhx50WRm54994wdq18t4COea3bQFWGU5u3W/YOQwaE9udSjupVYYTtS5sxoMLW2CSsutGAqpEwBazPqGHWH8hdOLLA7GzyP+IcgwJl+6aEtW9bu1AnqhzwYq8KgQuK17ZdyT/od9p106n38oQ11Z7fS9xLmxVvJzS2FzQPl4h/3X2ejvQTSPLTeNg+IbGFB6JPlG2WGcqaAYd+6f95Ly//mfSzevco9SvoYH1lann//GJGXbyM0G7NUd/SKCNPr/EpyNkG6+iq7ie6p1CATVExOqoQUSsYy2ygZGpNZResLn2xH5wAPUDKGuqSJmlfByC8Odwwlz8GsBi02uT5cry0XZf5tbeUOzpILzlGU";
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

    visionGuides = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

    red1 = visionGuides.get(3);
    red2 = visionGuides.get(1);
    blue1 = visionGuides.get(0);
    blue2 = visionGuides.get(2);

    red1.setName("Red1Gears");
    red2.setName("Red2Tools");
    blue1.setName("Blue1Wheels");
    blue2.setName("Blue2Legos");

    /** For convenience, gather together all the trackable objects in one easily-iterable collection */
    allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(visionGuides);

    OpenGLMatrix redTarget1LocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth / 2, -mmFTCFieldWidth / 12, 0)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));
    red1.setLocation(redTarget1LocationOnField);

    OpenGLMatrix redTarget2LocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth / 2, mmFTCFieldWidth / 4, 0)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));
    red2.setLocation(redTarget2LocationOnField);

    OpenGLMatrix blueTarget1LocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth / 12, mmFTCFieldWidth / 2, 0)
            .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 0, 0));
    blue1.setLocation(blueTarget1LocationOnField);

    OpenGLMatrix blueTarget2LocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth / 4, mmFTCFieldWidth / 2, 0)
            .multiplied(Orientation.getRotationMatrix(
                          /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 0, 0));
    blue2.setLocation(blueTarget2LocationOnField);

    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(mmBotWidth/2,0,0)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, -90, 0, 0));

    ((VuforiaTrackableDefaultListener)red1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    ((VuforiaTrackableDefaultListener)red2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    ((VuforiaTrackableDefaultListener)blue1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    ((VuforiaTrackableDefaultListener)blue2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

  }

  public OpenGLMatrix getAlignment(String guide) {
    VuforiaTrackable target = null;
    switch(guide) {
      case "gears":
        target = red1;
        break;
      case "tools":
        target = red2;
        break;
      case "wheels":
        target = blue1;
        break;
      case "legos":
        target = blue2;
        break;
      default:
        target = red1;
    }
    return ((VuforiaTrackableDefaultListener)target.getListener()).getPose();
  }

  public OpenGLMatrix getLocation() {
    OpenGLMatrix loc = null;

    for (VuforiaTrackable trackable : allTrackables) {
      /**
       * getUpdatedRobotLocation() will return null if no new information is available since
       * the last time that call was made, or if the trackable is not currently visible.
       * getRobotLocation() will return null if the trackable is not currently visible.
       */

      OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

      if (robotLocationTransform != null) {
        loc = robotLocationTransform;
      }
    }
    return loc;
  }

  /**
   *
   * @param loc Relative location as a matrix
   * @return location in 3 dimensions, {x, y, z} with the target at the origin.
   */
  public float[] getPosition(OpenGLMatrix loc) {
    VectorF pos = loc.getTranslation();
    return pos.getData();
  }

  public float getHeading(OpenGLMatrix loc) {
    Orientation o = Orientation.getOrientation(loc, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    return o.secondAngle;
  }

  public void activate() {
    visionGuides.activate();
  }
  public void deactivate() {
    visionGuides.deactivate();
  }
}
