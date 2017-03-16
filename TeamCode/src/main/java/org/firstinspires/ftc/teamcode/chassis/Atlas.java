package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FtcUtil;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

/**
 * Created by student on 2/3/17.
 */
public class Atlas {
    HardwareMap hwMap;
    DcMotor FL; // v1
    DcMotor FR; // v2
    DcMotor BL; // v3
    DcMotor BR; // v4

    DcMotor choo;
    DcMotor pickup;
    public DcMotor lift;

    Servo transfer;
    Servo pusher;
    public Servo foot;

    DigitalChannel chooLimit;

    AnalogInput dist; // center ultrasonic sensor (currently unplugged to avoid sonic interference)
    AnalogInput distF; // front ultrasonic sensor
    AnalogInput distB; // rear ultrasonic sensor
    OpticalDistanceSensor ods; // left line sensor
    OpticalDistanceSensor odsr; // right line sensor
    public Gyro imu;

    public ColorSensor colorSensor;

    /*
     * Values for transfer. Adjust these to change speed of transfer/servo positions in both telop and auto.
     */
    public final double LOAD_POSITION = .0;
    public final double REST_POSITION = .5;
    public final double STEP_SIZE = .02;
    public final int DELAY_TIME = 100;

    /*
     * Values for top foot.
     */

    public final double FOOT_IN = .62;
    public final double FOOT_UP = .06;
    public final double FOOT_DOWN = .29;

    public void init(HardwareMap ahwMap, boolean initSensors) {
        hwMap = ahwMap;

        FL = hwMap.dcMotor.get("FL");
        BL = hwMap.dcMotor.get("BL");
        FR = hwMap.dcMotor.get("FR");
        BR = hwMap.dcMotor.get("BR");

        pickup = hwMap.dcMotor.get("pickup");
        choo = hwMap.dcMotor.get("choo");
        choo.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = hwMap.dcMotor.get("lift");
        transfer = hwMap.servo.get("transfer");
        pusher = hwMap.servo.get("pusher");
        foot = hwMap.servo.get("foot");
        foot.setPosition(FOOT_IN);

        transfer.setPosition(REST_POSITION);
        pushStop();

        chooLimit = hwMap.digitalChannel.get("choo limit");
        chooLimit.setMode(DigitalChannelController.Mode.INPUT);

        dist = hwMap.analogInput.get("dist");
        distF = hwMap.analogInput.get("distF");
        distB = hwMap.analogInput.get("distB");
        ods = hwMap.opticalDistanceSensor.get("ods");
        odsr = hwMap.opticalDistanceSensor.get("odsr");
        if (initSensors) { // initializing the gyro takes time, so only intialize it if we need it.
            imu = new Gyro(hwMap.get(BNO055IMU.class, "imu"));
        }

        colorSensor = hwMap.colorSensor.get("mr");
        colorSensor.enableLed(false); // turn off LED on color sensor so it can see the light emitted by the beacons.

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init(HardwareMap ahwMap) {
        init(ahwMap, true);
    }

    /**
     *
     * @return true if the limit switch is in contact with the catapult.
     */
    public boolean catapultLoaded() {
        return !chooLimit.getState();
    }

    /**
     * Aggregate sensor data from both ultrasonic sensors to get the distance away from the wall.
     * @return Approximate distance from the wall in inches.
     */
    public double getDistance() {
        return (getFrontDistance() + getBackDistance())/2;
    }

    /**
     *
     * @return Distance in inches from front sensor to wall.
     */
    public double getFrontDistance() {
        return distF.getVoltage() * 98;
    }

    /**
     *
     * @return Distance in inches from back sensor to wall.
     */
    public double getBackDistance() {
        return distB.getVoltage() * 98;
    }

    /**
     *
     * @return True if left line sensor reads above a set brightness threshold.
     */
    public boolean isOnLinel() {
        return ods.getLightDetected() > .5;
    }

    /**
     *
     * @return True if right line sensor reads above a set brightness threshold.
     */
    public boolean isOnLiner() {
        return odsr.getLightDetected() > .5;
    }

    /**
     * Drive in a certain direction with a mecanum chassis
     * @param pow Base power (magnitude)
     * @param angle Angle to drive towards
     * @param rot speed of rotation
     */
    public void move(double pow, double angle, double rot) {
//        pow = FtcUtil.motorScale(pow);
        rot = FtcUtil.motorScale(rot);

        // Adding PI/4 ensures that 0 degrees is straight ahead
        double vx = pow*Math.cos(angle+Math.PI/4);
        double vy = pow*Math.sin(angle+Math.PI/4);

        double[] V = {vx+rot, vy-rot, vy+rot, vx-rot}; // contains motor powers for each motor.

    /*
     * because of adding/subtracting rotation, these numbers could be between [-2,2].
     * To get around this, find the maximum motor power, and divide all of them by that
     * so that the proportions stay the same but now it's between [-1,1].
     */

        // find max
        double m = 0.0;
        for (double v : V)
            if (Math.abs(v) > m)
                m = v;

        double mult = Math.max(Math.abs(pow), Math.abs(rot)); // If we're just rotating, pow will be 0
        // adjust values, still keeping power in mind.
        if (m != 0) // if the max power isn't 0 (can't divide by 0)
            for(int i = 0; i < V.length; i++)
                V[i] = Math.abs(mult) * (V[i]/Math.abs(m));

        // finally, set motor powers.
        FL.setPower(FtcUtil.motorScale(V[0]));
        FR.setPower(FtcUtil.motorScale(V[1]));
        BL.setPower(FtcUtil.motorScale(V[2]));
        BR.setPower(FtcUtil.motorScale(V[3]));

    }

    int pos = 0;

    /**
     * Get encoder ticks driven.
     * @return encoder ticks
     */
    public int getTicks() {
        return BR.getCurrentPosition() - pos;
    }

    /**
     * Reset encoder ticks for next movement.
     */
    public void resetTicks() {
        pos = BR.getCurrentPosition();
    }
    public void runPickup(double pow) {
        pickup.setPower(pow);
    }

    public void drive(double fl, double bl, double fr, double br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);
    }

    void moveLeft() {
        drive(-1, 1, 1, -1);
    }

    void moveRight() {

    }

    public void stopMotors() {
        drive(0, 0, 0, 0);
    }

    public void transervo(double chamberpos) { transfer.setPosition(chamberpos); }

    public void runChoo(double pow) {
        choo.setPower(pow);
    }

    double integral = 0.0; // Accumulation of error over time. Used in PID controller.
    double lastError = 0.0;
    /**
     * Use PID in order to move directly in the desired angle, without rotation.
     * @param pow Motor power (approximate speed)
     * @param angle Desired angle
     * @param actual Orientation/rotation measurement
     * @param target Desired orientation/rotation
     */
    public void moveStraight(double pow, double angle, double actual, double target) {
        double error = actual - target;
        integral += error;

        double change = (error - lastError);
        lastError = error;

        double PID = getKp()*error + getKi()*integral + getKd()*change;

        double rot = FtcUtil.motorScale(PID);
        this.move(pow, angle, rot);
    }

    /**
     * Use PID to move without rotation, when the target angle is 0.
     */
    public void moveStraight(double pow, double angle, double actual) {
        this.moveStraight(pow, angle, actual, 0);
    }

    /*
     * Constant values for PID.
     */
    double getKp() {
        return -0.02;
    }
    double getKi() {
        return 0.0;
    }
    double getKd() {
        return 0.0;
    }

    public void pushOut() {
        pusher.setPosition(0);
    }
    public void pushIn() {
        pusher.setPosition(1);
    }
    public void pushStop() {
        pusher.setPosition(.520);
    }
    public void push() throws InterruptedException{
        pushOut();
        Thread.sleep(1500);
        pushStop();
        Thread.sleep(200);
        pushIn();
        Thread.sleep(1500);
        pushStop();
    }

}
