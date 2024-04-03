package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Variables {
    public LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.

    public static double wrist = 0;
    public static double OutsidePixel = 0;
    public static double InsidePixel = 0;
    public static double Intake = 0;
    public static double p6servo = 0;
    public static double ScissorLeft = 0;
    public static double ScissorRight = 0;
    public static double imu = 0;

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


    //Lift Varables
    public double LeftLiftPower = 0;
    public double RightLiftPower = 0;

    //Servo variables
    public static double dWristIn = 0.49;
    public static double dWristDeliver = 0.77;
    public static double dOutsideIn = 0.43;
    public static double dOutside1Pixel = 0.65;
    public static double dOutside2Pixel = 1;
    public static double dInsideIn = 0;
    public static double dInsideHold = 0.8;

    public static double dIntakeSpeed = 0.9;

    public static double dDronePos = 0.5;

    public int iDeliveryState = 0;


    public Variables(LinearOpMode opmode) {
        this.opMode = opmode;
    }
}
