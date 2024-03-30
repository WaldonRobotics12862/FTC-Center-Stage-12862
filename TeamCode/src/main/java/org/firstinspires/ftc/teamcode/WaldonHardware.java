package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class WaldonHardware {

    public LinearOpMode opMode = null;   // gain access to methods in the calling OpMode.
    public HardwareMap WaldonHMap;

    public VoltageSensor controlHubBatteryVoltage;
    public VoltageSensor expansionHubBatteryVoltage;

    public LynxModule controlHub;
    public LynxModule expansionHub;

    // First, let's define all of the hardware elements that we will interface with.
    // There are two constant rotation servos
    public CRServo intake_servo_1;
    public CRServo intake_servo_2;

    //5 more regular servos
    public Servo wrist;
    public Servo InsidePixel;
    public Servo p6servo;
    public Servo OutsidePixel;
    public Servo drone;

    // 7 total motors
    public DcMotor leftfront_drive;
    public DcMotor leftback_drive;
    public DcMotor rightback_drive;
    public DcMotor rightfront_drive;
    public DcMotor ScissorLeft;
    public DcMotor ScissorRight;
    public DcMotor Intake;

    // Now for all of our sensors: 3 distance, 1 IMU and 1 color
    public BNO055IMU imu;
    public DistanceSensor leftDistanceSensor;  //not used in teleop but defined anyways
    public DistanceSensor centerDistanceSensor; //not used in teleop but defined anyways
    public DistanceSensor rightDistanceSensor; //not used in teleop but defined anyways
    public ColorSensor p6Color; //not used in teleop but defined anyways

    // Second, let's create some variables that we'll use as we write our code.
    // MAKE SURE TO INITALIZE THE VARIABLES, DON'T JUST DECLARE THEM AS EMPTY UNLESS A CONTAINE

    BNO055IMU.Parameters imuParameters;  // imuParameters is a container, so we'll have to initalize that upon startup

    //AprilTagProcessor myAprilTagProcessor; // we're not using AprilTags in this version, so we don't need this

    //Drive variables
    double dTurn = 0;
    double dDrive = 0;
    double dGamePadDegree = 0;
    double dMovement = 0;
    double dForward = 0;
    double dStrafe = 0;
    double dDriveScale = 1;

    double dLFDrivePower = 0;
    double dLBDrivePower = 0;
    double dRFDrivePower = 0;
    double dRBDrivePower = 0;

    double dDenominator  = 1; // don't set a denominator to 0 or we'll get a divide by 0 error

    //Lift Varables
    double LeftLiftPower = 0;
    double RightLiftPower = 0;

    //Servo variables
    double dWristIn = 0.49;
    double dWristDeliver = 0.77;
    double dOutsideIn = 0.43;
    double dOutside1Pixel = 0.65;
    double dOutside2Pixel = 1;
    double dInsideIn = 0;
    double dInsideHold = 0.8;

    double dDronePos = 0.5;

    int iDeliveryState = 0;

    double dInsidePixelServo = dInsideHold;
    double dWristServo = dWristIn;
    double dOutsidePixel = dOutsideIn;
    double dIntakeSpeed = 0;
    double dP6ServoPosition = 0;
    public WaldonHardware(HardwareMap baseHMap, LinearOpMode opmode) {
        this.opMode = opmode;
        this.WaldonHMap = baseHMap;
    }

    public void initialize(){
        opMode.telemetry.addData("Status", "detecting...");

        controlHubBatteryVoltage = WaldonHMap.get(VoltageSensor.class, "Control Hub");
        expansionHubBatteryVoltage = WaldonHMap.get(VoltageSensor.class, "Expansion Hub 2");
        controlHub = WaldonHMap.get(LynxModule.class, "Control Hub");
        expansionHub = WaldonHMap.get(LynxModule.class, "Expansion Hub 2");


        //2 Constant Rotation Servos
        intake_servo_1 = WaldonHMap.get(CRServo.class, "intake1");
        intake_servo_2 = WaldonHMap.get(CRServo.class, "intake2");

        //5 Regular Servos
        wrist = WaldonHMap.get(Servo.class, "wrist");
        InsidePixel = WaldonHMap.get(Servo.class, "InsidePixel");
        p6servo = WaldonHMap.get(Servo.class, "p6servo");
        OutsidePixel = WaldonHMap.get(Servo.class, "OutsidePixel");
        drone = WaldonHMap.get(Servo.class, "drone");

        //7 Motors
        leftfront_drive = WaldonHMap.get(DcMotor.class, "leftfront_drive");
        leftback_drive = WaldonHMap.get(DcMotor.class, "leftback_drive");
        rightback_drive = WaldonHMap.get(DcMotor.class, "rightback_drive");
        rightfront_drive = WaldonHMap.get(DcMotor.class, "rightfront_drive");
        ScissorLeft = WaldonHMap.get(DcMotor.class, "ScissorLeft");
        ScissorRight = WaldonHMap.get(DcMotor.class, "ScissorRight");
        Intake = WaldonHMap.get(DcMotor.class, "Intake");

        //Sensors
        imu = WaldonHMap.get(BNO055IMU.class, "imu");
        leftDistanceSensor = WaldonHMap.get(DistanceSensor.class, "leftDistanceSensor");
        centerDistanceSensor = WaldonHMap.get(DistanceSensor.class, "centerDistanceSensor");
        rightDistanceSensor = WaldonHMap.get(DistanceSensor.class, "rightDistanceSensor");
        p6Color = WaldonHMap.get(ColorSensor.class, "P6Color");

    }

}
