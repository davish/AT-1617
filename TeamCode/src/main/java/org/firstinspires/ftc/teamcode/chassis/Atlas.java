package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FtcUtil;

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

    DigitalChannel chooLimit;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        FL = hwMap.dcMotor.get("FL");
        BL = hwMap.dcMotor.get("BL");
        FR = hwMap.dcMotor.get("FR");
        BR = hwMap.dcMotor.get("BR");

        pickup = hwMap.dcMotor.get("pickup");
        choo = hwMap.dcMotor.get("choo");
        lift = hwMap.dcMotor.get("lift");
        transfer = hwMap.servo.get("transfer");

        chooLimit = hwMap.digitalChannel.get("choo limit");
        chooLimit.setMode(DigitalChannelController.Mode.INPUT);


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean catapultLoaded() {
        return chooLimit.getState();
    }


    /**
     * Drive in a certain direction with a mecanum chassis
     * @param pow Base power (magnitude)
     * @param angle Angle to drive towards
     * @param rot speed of rotation
     */
    public void move(double pow, double angle, double rot) {
        pow = FtcUtil.motorScale(pow);
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
    public int getTicks() {
        return BL.getCurrentPosition() - pos;
    }
    public void resetTicks() {
        pos = BL.getCurrentPosition();
    }
    public void runPickup(double pow) {
        pickup.setPower(pow);
    }

    void drive(float fl, float bl, float fr, float br) {
        FL.setPower(fl);
        BL.setPower(bl);
        FR.setPower(fr);
        BR.setPower(br);

    }

    void moveLeft() {
        drive(-1,1,1,-1);
    }

    void moveRight() {

    }
    public void transervo(double chamberpos) { transfer.setPosition(chamberpos); }

    public void runChoo(double pow) {
        choo.setPower(pow);
    }

}
