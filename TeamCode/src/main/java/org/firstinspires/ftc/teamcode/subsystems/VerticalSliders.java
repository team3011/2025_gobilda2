package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//read this to learn about PID and feedforward
//https://www.ctrlaltftc.com/the-pid-controller

@Config
public class VerticalSliders {
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;
    private int lastPosition = 0;
    private int targetPosition = 0;
    private PIDController controller;
    public static double kP = 0.02;
    public static double kI = 0.001;
    public static double kD = 0.0003;
    public static double kG = 0.015;
    public static float convertTicksToMillimeters = .225f; // 225mm/1000ticks = .225
    private boolean resetFlag = false;
    private double lastTimeWeCheck = 0;
    public static int minimumEncoderMovement = 10;
    public static int howOftenWeCheck = 500; //ms
    public static int maximumMilliamps = 1000;
    private boolean softLimitEngaged = false;
    public static double maxPower = .4;
    public static double minimumSpeed = 0.1;
    private boolean goingUp = false;
    private boolean holdingPosition = true;
    private int targetPositionMM;


    public VerticalSliders(HardwareMap hardwareMap){
        this.rightMotor = hardwareMap.get(DcMotorEx.class,"vertiRight");
        this.rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor = hardwareMap.get(DcMotorEx.class,"vertiLeft");
        this.leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.controller = new PIDController(this.kP, this.kI, this.kD);
    }

    public double getCurrent(int motor){
        if (motor == 0) {
            return Math.round(this.leftMotor.getCurrent(CurrentUnit.MILLIAMPS)/10)*10;
        }
        return Math.round(this.rightMotor.getCurrent(CurrentUnit.MILLIAMPS)/10)*10;
    }

    //this should return the current position in mm above starting position
    public int getPositionMillimeter(){
        //creating the result as a float needs to be done
        //since we dont care about precision enough we just use a float :thumbsup:
        float result = this.rightMotor.getCurrentPosition() * convertTicksToMillimeters;
        return Math.round(result);
    }

    //returns position in TICKS
    public int getPosition(){
        return this.rightMotor.getCurrentPosition();
    }

    //mm is the distance in millimeters you want the sliders to move
    public void setPosition(int mm){
        //are we trying to move?
        if (this.targetPositionMM != mm) {
            this.holdingPosition = false;
            //when we go down we need to reset
            if (mm > 0) {
                this.resetFlag = true;
            }

            if (mm - this.targetPosition > 0) {
                this.goingUp = true;
            } else {
                this.goingUp = false;
            }

            if (mm > 940) {
                mm = 940;
            }

            this.targetPositionMM = mm;
            // mm divided by ratio = mm multiplied by ratio^-1
            // ^ for my sanity
            float notInteger = mm / convertTicksToMillimeters;
            mm = Math.round(notInteger);
            this.softLimitEngaged = false;
            this.targetPosition = mm;
        }
    }

    public void update(double lim, Telemetry dashboardTelemetry){
        if (!this.holdingPosition) {
            double pid = 0;
            //get the current position of the sliders
            int currentPosition = getPosition();

            //calc the speed the motor should move
            pid = this.controller.calculate(currentPosition, this.targetPosition);
            pid = limiter(pid, maxPower);
            pid += VerticalSliders.kG;
            lastPosition = currentPosition;


            if (targetPosition != 0 && Math.abs(pid)< minimumSpeed && Math.abs(currentPosition-lastPosition)<10) {
                this.holdingPosition = true;
                if (targetPosition > 100) {
                    pid = VerticalSliders.kG;
                }
            }

            if (this.resetFlag && !this.goingUp && currentPosition < 400) {
                if (this.getCurrent(1) > maximumMilliamps){
                    this.resetFlag = false;
                    this.holdingPosition = true;
                    pid = 0;
                    this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    this.targetPosition = 0;
                } else {
                    pid = -0.2;
                }
            }

            this.rightMotor.setPower(pid);
            this.leftMotor.setPower(pid);

            dashboardTelemetry.addData("current position in ticks", currentPosition);
            dashboardTelemetry.addData("pid output", pid);
        }

        dashboardTelemetry.addData("reset Flag", resetFlag);
        dashboardTelemetry.addData("going up", goingUp);
        dashboardTelemetry.addData("milliamps", this.getCurrent(1));
        dashboardTelemetry.addData("holding pos", this.holdingPosition);
        dashboardTelemetry.addData("target position in ticks", this.targetPosition);


        dashboardTelemetry.update();
    }

    /**
     * this will limit the input to a range of -limiter to limiter
     * @param input the value to be limited
     * @param limiter the max value the input can be
     * @return the limited input
     */
    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }




}