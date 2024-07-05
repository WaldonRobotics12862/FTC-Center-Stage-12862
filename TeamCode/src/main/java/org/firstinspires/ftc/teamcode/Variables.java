package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Variables {
    public LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    public static double p6servo = 0;

    public static double wristHome = 0.49;
    public static double wristDelivery = 0.9;

    public static double OutsidePixelHome = 0;
    public static double OutsidePixelSingle = 0.5;
    public static double OutsidePixelDouble = 1.0;

    public static double InsidePixelHome = 0;
    public static double InsidePixelSingle = 0.5;
    public static double InsidePixelDouble = 1;

    public static double blue_threshold = 400;
    public static double red_threshold = 320;

    double WheelCircomference = 3.77953 * Math.PI;
    double CountsPerMotorRev = 537.7;
    public double countsPerInch = CountsPerMotorRev / WheelCircomference;

    //imu
    public BNO055IMU.Parameters imuParameters;

    //Drive Variables
    public double dTurn = 0;
    public double dDrive = 0;
    public double dGamePadDegree = 0;
    public double dMovement = 0;
    public double dForward = 0;
    public double dStrafe = 0;
    public double dDriveScale = 1;
    public double dDenominator = 1;
    public double dLFDrivePower = 0;
    public double dLBDrivePower = 0;
    public double dRFDrivePower = 0;
    public double dRBDrivePower = 0;

    //Lift Variables
    public double LeftLiftPower = 0;
    public double RightLiftPower = 0;

    //Intake
    public double dIntakeSpeed = 0;

    //Servo variables
    public static double dWristIn = 0.49;
    public static double dWristDeliver = 0.77;
    public static double dOutsideIn = 0.43;
    public static double dOutside1Pixel = 0.65;
    public static double dOutside2Pixel = 1;
    public static double dInsideIn = 0;
    public static double dInsideHold = 0.8;

    public static double dDronePos = 0.5;

    public int iDeliveryState = 0;

    public Variables(LinearOpMode opmode) {
        this.opMode = opmode;
    }
}
