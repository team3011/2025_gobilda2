package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class HorizontalArm {
    public static double scanPos = 0.64;
    public static double pickupPos = 0.84;
    public static double transferPos= 0.4;
    public static double stowPos= 0.5;
    public static int timePickupToTransfer = 1000;
    public static int timeTransferToScan = 500;
    public static int timeScantoPickup = 500;
    public static int timeTransferToStow = 125;


    private Servo leftElbow;
    private Servo rightElbow;

    private double target;
    private double currentPos;
    private int time;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2= new ElapsedTime();
    private double moveIncrement;


    public HorizontalArm(@NonNull HardwareMap hardwareMap){
        leftElbow = hardwareMap.get(Servo.class, "leftArm");
        rightElbow = hardwareMap.get(Servo.class, "rightArm");
        rightElbow.setDirection(Servo.Direction.REVERSE);
    }

    public void toScanPos(){
        if (currentPos == stowPos || currentPos == transferPos) {
            setTarget(scanPos, timeTransferToScan);
        }
    }

    public void toPickupPos(){
        if (currentPos == scanPos) {
            setTarget(pickupPos, timeScantoPickup);
        }
    }

    public void toStowPos(){
        if (currentPos == transferPos) {
            setTarget(stowPos, timeTransferToStow);
        }

    }

    public void toTransferPos(){
        if (currentPos == pickupPos) {
            setTarget(transferPos, timePickupToTransfer);
        }
    }

    public void goToStartPos(){
        setPosition(transferPos);
    }

    public void setPosition(double input){
        this.leftElbow.setPosition(input);
        this.rightElbow.setPosition(input);
        this.currentPos = input;
        this.target = input;
    }

    /**
     * Will set desired target for the servo and the time it will take to get there
     * servo will move gradually to target in the update function
     *
     * @param input value between 0 and 1, make sure servo can handle it
     * @param t time it will take to go from current location to new location in milliseconds,
     *          a value of 0 will move at the max servo speed
     */
    public void setTarget(double input, int t) {
        if (input != this.currentPos) {
            this.target = input;
            this.time = t;
            this.timer.reset();
            this.moveIncrement = (input - this.currentPos) / t;
        }
    }

    /**
     * standard update function that will move the wrist if not at the desired location
     */
    public void update(){

            double duration = this.timer.milliseconds();
            if (Math.abs(this.currentPos - this.target) > .01 && duration < this.time) {
                double temp = this.currentPos + duration * this.moveIncrement;
                this.leftElbow.setPosition(temp);
                this.rightElbow.setPosition(temp);
            } else {
                this.leftElbow.setPosition(this.target);
                this.rightElbow.setPosition(this.target);
                this.currentPos = this.target;
            }

    }
}
