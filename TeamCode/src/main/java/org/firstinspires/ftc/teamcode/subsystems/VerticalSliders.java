package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class VerticalSliders {
    private DcMotorEx rightMotor;
    private DcMotorEx leftMotor;

    public VerticalSliders(HardwareMap hardwareMap){
        this.rightMotor = hardwareMap.get(DcMotorEx.class,"vertiRight");
        this.rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftMotor = hardwareMap.get(DcMotorEx.class,"vertiLeft");
        this.leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public int getHeight(){
        return this.rightMotor.getCurrentPosition();
    }
}
