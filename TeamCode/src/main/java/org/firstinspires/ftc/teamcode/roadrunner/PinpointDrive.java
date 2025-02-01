package org.firstinspires.ftc.teamcode.roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;


/**
 * Experimental extension of MecanumDrive that uses the Gobilda Pinpoint sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by Gobilda (Base 10 Assets, LLC)
 * Unless otherwise noted, comments are from Gobilda
 */
@Config
public class PinpointDrive extends MecanumDrive {
    public static double correctionMultiplier = 1;
    public static double ANGULAR_TOLERANCE_DEGREES = 2;
    public static double rotMulti = 1.1;



    public static Params PARAMS = new Params();
    public GoBildaPinpointDriverRR pinpoint;
    private Pose2d lastPinpointPose = pose;
    private double headingToMaintain = 0;
    public PinpointDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        FlightRecorder.write("PINPOINT_PARAMS", PARAMS);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");


        // RR localizer note: don't love this conversion (change driver?)
        pinpoint.setOffsets(DistanceUnit.MM.fromInches(PARAMS.xOffset), DistanceUnit.MM.fromInches(PARAMS.yOffset));


        pinpoint.setEncoderResolution(PARAMS.encoderResolution);


        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);


       /*
       Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
       The IMU will automatically calibrate when first powered on, but recalibrating before running
       the robot is a good idea to ensure that the calibration is "good".
       resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
       This is recommended before you run your autonomous, as a bad initial calibration can cause
       an incorrect starting value for x, y, and heading.
        */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        pinpoint.setPosition(pose);

    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override otos pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            pinpoint.setPosition(pose);
        }
        pinpoint.update();
        pose = pinpoint.getPositionRR();
        lastPinpointPose = pose;


        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }


        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE", new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS", pinpoint.getDeviceStatus());


        return pinpoint.getVelocityRR();
    }

    public GoBildaPinpointDriverRR getPinpoint() {
        return pinpoint;
    }

    public static class Params {

        //limits how fast human can rotate robot, decrease to slow down rotation
        public double rotation_multi = 0.5;
        //acceptable angle tolerance in radians
        public double ANGULAR_TOLERANCE = Math.PI/90;

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        //These are tuned for 3110-0002-0001 Product Insight #1
        // RR localizer note: These units are inches, presets are converted from mm (which is why they are inexact)
        public double xOffset = 2;
        public double yOffset = -4.5;


        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.


        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD;


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    // for debug logging
    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;


        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }

    //***************************************************
    //pulled from 2024 drive code to integrate Pinpoint
    //***************************************************

    //returns the current heading of the robot in RAD
    public double calcYaw(Telemetry db) {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        return Math.round(pinpoint.getHeading()*10)/10.0;
    }

    //determines the shortest path to desired angle in degrees
    public double figureOutWhatIsShorter(double reading) {
        double result;
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 360;
        } else {
            oppositeButEqualReading = reading + 360;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);
        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return -result;
    }

    //return a value that is below or equal to the limit
    private double limiter(double input, double lim) {
        //this will limit the pid to a range of -1 to 1
        if (input > lim) {
            input = lim;
        } else if (input < -lim) {
            input = -lim;
        }
        return input;
    }

    public void setHeadingToMaintain(double input){
        this.headingToMaintain = input;
    }

    public void drive2(double x, double y, double rx, Telemetry db){
        double robotHeadingRAD = calcYaw(db);
        double robotHeadingDEG = Math.toDegrees(robotHeadingRAD);
        db.addData("yaw rads",robotHeadingRAD);

        if(rx == 0){
            //we're trying to maintain our current heading
            //calc the shortest deviation to target heading in degrees
            double shorter = this.figureOutWhatIsShorter(robotHeadingDEG);
            db.addData("shorter",shorter);
            //check if we are within tolerance
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) < ANGULAR_TOLERANCE_DEGREES;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                //this means we are moving
                if (Math.abs(y) > 0 || Math.abs(x) > 0) {
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 0.70;
                    } else {
                        rotSpeed = correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                } else {
                    //this means we are not moving but not pointing in the right direction
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 0.70;
                    } else {
                        rotSpeed = 2 * correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                }
            }
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeadingDEG;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(-robotHeadingRAD) - y * Math.sin(-robotHeadingRAD);
        double rotY = x * Math.sin(-robotHeadingRAD) + y * Math.cos(-robotHeadingRAD);
        rotX = rotX * rotMulti;

        //double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        //double frontLeftPower = (y+x+rx) / denominator;
        //double backLeftPower = (y - x + rx) / denominator;
        //double frontRightPower = (y-x-rx) / denominator;
        //double backRightPower = (y + x - rx) / denominator;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx)  / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx)   / denominator;
        double backRightPower = (rotY + rotX - rx)  / denominator;

        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);

        db.addData("rx",rx);
        db.addData("fLP",frontLeftPower);
        db.addData("fRP",frontRightPower);
        db.addData("rLP",backLeftPower);
        db.addData("rRP",backRightPower);


    }

    //code taken from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
    public void drive(double leftStickX, double leftStickY, double rightStickX, boolean fromAuto, Telemetry db) {
        double x = leftStickX;
        double y = leftStickY; // Counteract imperfect strafing
        double rx = rightStickX * PARAMS.rotation_multi; //what way we want to rotate
        double robotHeadingRAD = calcYaw(db);
        double robotHeadingDEG = Math.toDegrees(robotHeadingRAD);


        if(rx == 0){
            //we're trying to maintain our current heading
            //calc the shortest deviation to target heading in degrees
            double shorter = this.figureOutWhatIsShorter(robotHeadingDEG);
            db.addData("shorter",shorter);
            //check if we are within tolerance
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) < ANGULAR_TOLERANCE_DEGREES;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                //this means we are moving
                if (Math.abs(y) > 0 || Math.abs(x) > 0) {
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 0.70;
                    } else {
                        rotSpeed = correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                } else {
                    //this means we are not moving but not pointing in the right direction
                    double rotSpeed = Math.abs(shorter);
                    if (rotSpeed > 20) {
                        rotSpeed = 0.70;
                    } else {
                        rotSpeed = 2 * correctionMultiplier * rotSpeed * rotSpeed / 800.0;
                    }
                    rx = limiter(shorter, rotSpeed);
                }
            }
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeadingDEG;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(-robotHeadingRAD) + y * Math.sin(-robotHeadingRAD);
        double rotY = x * Math.sin(-robotHeadingRAD) - y * Math.cos(-robotHeadingRAD);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx)  / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx)   / denominator;
        double backRightPower = (rotY + rotX - rx)  / denominator;

        db.addData("rx",rx);
        db.addData("fL",frontLeftPower);
        db.addData("fR",frontRightPower);
        db.addData("rL",backLeftPower);
        db.addData("rR",backRightPower);

        //leftFront.setPower(frontLeftPower);
        //rightFront.setPower(frontRightPower);
        //leftBack.setPower(backLeftPower);
        //rightBack.setPower(backRightPower);

        double fL_velocity = map(frontLeftPower,-1,1,-340, 340);
        double fR_velocity = map(frontRightPower,-1,1,-340, 340);
        double rL_velocity = map(backLeftPower,-1,1,-340, 340);
        double rR_velocity = map(backRightPower,-1,1,-340, 340);

        leftFront.setVelocity(fL_velocity,AngleUnit.DEGREES);
        rightFront.setVelocity(fR_velocity,AngleUnit.DEGREES);
        leftBack.setVelocity(rL_velocity,AngleUnit.DEGREES);
        rightBack.setVelocity(rR_velocity,AngleUnit.DEGREES);

    }

    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}