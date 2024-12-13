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
    private double lastCheck = 0;
    private int checkDelay = 50; //in ms
    private PIDController controller;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;


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

    //up right is positive power
    //up left is positive power
    public void manualPower(double input){
        this.rightMotor.setPower(input);
        this.leftMotor.setPower(input);
    }

    public double getCurrent(int motor){
        if (motor == 0) {
            return this.leftMotor.getCurrent(CurrentUnit.MILLIAMPS);
        }
        return this.rightMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    //this should return the current position in mm above starting position
    public int getPosition(){
        return this.rightMotor.getCurrentPosition();
    }

    //mm is the distance in millimeters you want the sliders to move
    public void setPosition(int mm){
        this.targetPosition = mm;
    }


    public void update(double lim, Telemetry dashboardTelemetry, ElapsedTime runTime){
        double power = 0;
        this.controller.setPID(VerticalSliders.kP, VerticalSliders.kI, VerticalSliders.kD);
        double pid = this.controller.calculate(getPosition(),this.targetPosition);
        pid = limiter(pid, lim);
        this.rightMotor.setPower(pid);
        this.leftMotor.setPower(pid);
        dashboardTelemetry.addData("pid output", pid);
        dashboardTelemetry.addData("target position", this.targetPosition);
        dashboardTelemetry.addData("current position", getPosition());
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
