package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Teleop2023")
public class Teleop2023 extends LinearOpMode {
    // First, let's define all of the hardware elements that we will interface with.
    // There are two constant rotation servos
    private CRServo intake_servo_1;
    private CRServo intake_servo_2;

    //5 more regular servos
    private Servo wrist;
    private Servo InsidePixel;
    private Servo p6servo;
    private Servo OutsidePixel;
    private Servo drone;

    // 7 total motors
    private DcMotor leftfront_drive;
    private DcMotor leftback_drive;
    private DcMotor rightback_drive;
    private DcMotor rightfront_drive;
    private DcMotor ScissorLeft;
    private DcMotor ScissorRight;
    private DcMotor Intake;

    // Now for all of our sensors: 3 distance, 1 IMU and 1 color
    private BNO055IMU imu;
    private DistanceSensor leftDistanceSensor;  //not used in teleop but defined anyways
    private DistanceSensor centerDistanceSensor; //not used in teleop but defined anyways
    private DistanceSensor rightDistanceSensor; //not used in teleop but defined anyways
    private ColorSensor p6Color; //not used in teleop but defined anyways

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

     @Override
    public void runOpMode() {

        //**************************************************************//
        //                                                              //
        // Create a hardware map section to address all of our physical //
        // robot parts.  For each, we'll declare baseically a variable  //
        // and set it equal to the hareware map and call the class of   //
        // the part.  For instance, a motor or a servo.                 //
        //                                                              //
        // These should match the section before the main loop where we //
        // first declare these as variables to use                      //
        //                                                              //
        //**************************************************************//

        //2 Constant Rotation Servos
        intake_servo_1 = hardwareMap.get(CRServo.class, "intake1");
        intake_servo_2 = hardwareMap.get(CRServo.class, "intake2");

        //5 Regular Servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        InsidePixel = hardwareMap.get(Servo.class, "InsidePixel");
        p6servo = hardwareMap.get(Servo.class, "p6servo");
        OutsidePixel = hardwareMap.get(Servo.class, "OutsidePixel");
        drone = hardwareMap.get(Servo.class, "drone");

        //7 Motors
        leftfront_drive = hardwareMap.get(DcMotor.class, "leftfront_drive");
        leftback_drive = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightback_drive = hardwareMap.get(DcMotor.class, "rightback_drive");
        rightfront_drive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        ScissorLeft = hardwareMap.get(DcMotor.class, "ScissorLeft");
        ScissorRight = hardwareMap.get(DcMotor.class, "ScissorRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        centerDistanceSensor = hardwareMap.get(DistanceSensor.class, "centerDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        p6Color = hardwareMap.get(ColorSensor.class, "P6Color");

         // Now, we can create our main loop

         //**************************************************************//
         //                                                              //
         // The following is our initialize block of code where we set   //
         // up everything from motor directions to the initial servo     //
         // rotation position.                                           //
         //                                                              //
         //**************************************************************//

         // Any final set-up of things that we need to take care of, change direction of motors, etc.
        intake_servo_1.setDirection(CRServo.Direction.REVERSE);
        intake_servo_2.setDirection(CRServo.Direction.FORWARD);

        leftfront_drive.setDirection(DcMotor.Direction.FORWARD);
        leftback_drive.setDirection(DcMotor.Direction.FORWARD);
        rightback_drive.setDirection(DcMotor.Direction.REVERSE);
        rightfront_drive.setDirection(DcMotor.Direction.REVERSE);

        ScissorLeft.setDirection(DcMotor.Direction.FORWARD);
        ScissorRight.setDirection(DcMotor.Direction.FORWARD);

        leftback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ScissorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ScissorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initalize parameters that we couldn't just set values to
        init_IMU();

        // Initalize all of our servo positions.
        p6servo.setPosition(dP6ServoPosition);
        wrist.setPosition(dWristServo);
        InsidePixel.setPosition(dInsidePixelServo);
        OutsidePixel.setPosition(dOutsidePixel);

        // Wait for the match to begin.
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Strafe", dStrafe);
            telemetry.addData("Forward", dForward);
            telemetry.addData("Turn", dTurn);
            telemetry.update(); // Push telemetry to the Driver Station.

            GetDriverController();
            GetCoPilotController();

            Deliver_Mechanism();
            Intake();
            ScissorLift();
            P6();
            Drone();

            Drive();
        }
    }

    private void init_IMU(){
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParameters);
    }

    private double dGetZAxis() {
        //check which angle we are at according to the IMU.
        Orientation oAngle;
        oAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return oAngle.firstAngle;
    }

    private void GetDriverController(){
        //gamepad1.y		= open up P6
        //gamepad1.right_stick_x	= turn (inverse)
        //gamepad1.left_stick_y	= move (forward/reverse)
        //gamepad1.left_stick_x	= move (strafe)
        //gamepad1.left_bumper	= slow
        //gamepad1.right_bumper	= super slow
        //gamepad1.a		= reset IMU

        if (gamepad1.a){
            init_IMU();
        }
        if (gamepad1.y){
            dP6ServoPosition = 0.5;
        }
        if (gamepad1.left_bumper) {
            dDriveScale = 0.68;
        }
        if (gamepad1.right_bumper) {
            dDriveScale = 0.5;
        }
        // Calculate the drive parameters
        dTurn = (-gamepad1.right_stick_x * dDriveScale); // adding the (double) casts this as a double variable to match the dTurn.  Not sure if the right_stick_x is a double by default or not)
        dDrive = Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), 0, 1);  // dDrive = sqrt(Y^2 + X^2). the clipping function clips any values to be just between 0 and 1
        dGamePadDegree = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180; // calculate the angle by use of the arc tangent
        dMovement = dGamePadDegree - dGetZAxis();
        dStrafe = Math.cos(dMovement / 180 * Math.PI) * dDrive * dDriveScale;
        dForward = Math.sin(dMovement / 180 * Math.PI) * dDrive * dDriveScale;

        dDenominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(dForward) + Math.abs(dStrafe + Math.abs(dTurn)), 1));
        dLFDrivePower = ((dForward * Math.abs(dForward) + dStrafe * Math.abs(dStrafe)) + dTurn) / dDenominator;
        dRFDrivePower = ((dForward * Math.abs(dForward) - dStrafe * Math.abs(dStrafe)) - dTurn) / dDenominator;
        dLBDrivePower = ((dForward * Math.abs(dForward) + dStrafe * Math.abs(dStrafe)) - dTurn) / dDenominator;
        dLFDrivePower = ((dForward * Math.abs(dForward) - dStrafe * Math.abs(dStrafe)) + dTurn) / dDenominator;
    }

    private void GetCoPilotController(){
        //gamepad2.guide		= Launch Drone
        //gamepad2.dpad_down	= Lift down
        //gamepad2.dpad_up		= Lift Up
        //gamepad2.right_trigger	= Right Lift Down
        //gamepad2.left_trigger	= Left Lift Down
        //gamepad2.left_bumper	= Intake
        //gamepad2.right_bumper	= Outtake
        //gamepad2.b		=
        //gamepad2.a		=
        //gamepad2.x		=
        if (gamepad2.a){
            iDeliveryState = 0;
        }
        if (gamepad2.b){
            iDeliveryState = 1;
        }
        if (gamepad2.x){
            if (iDeliveryState == 2){
                iDeliveryState = 3;
            } else {
                iDeliveryState = 2;
            }
        }
        if (gamepad2.guide){
            dDronePos = 0.5;
        }
        if (gamepad2.dpad_down) {
            //set both lift powers to -1
            LeftLiftPower = -1;
            RightLiftPower = -1;
        } else if (gamepad2.dpad_up) {
            // set both lift powers to 1
            LeftLiftPower = 1;
            RightLiftPower = 1;
        } else {
            // set both lift powers to the trigger value so that we can gracefully lower them
            LeftLiftPower = -gamepad2.left_trigger;
            RightLiftPower = -gamepad2.right_trigger;
        }

        if (gamepad2.left_bumper){
            dIntakeSpeed = 1;
        } else if (gamepad2.right_bumper){
            dIntakeSpeed = -1;
        } else {
            dIntakeSpeed = 0;
        }
    }

    private void Deliver_Mechanism(){
        switch(iDeliveryState) {
            case 0: //reset
                wrist.setPosition(dWristIn);
                InsidePixel.setPosition(dInsideIn);
                OutsidePixel.setPosition(dOutsideIn);
                break;
            case 1: //Deliver setup
                wrist.setPosition(dWristDeliver);
                InsidePixel.setPosition(dInsideHold);
                OutsidePixel.setPosition(dOutsideIn);
                break;
            case 2: //Deliver pixel
                wrist.setPosition(dWristDeliver);
                InsidePixel.setPosition(dInsideHold);
                OutsidePixel.setPosition(dOutside1Pixel);
                break;
            case 3: // Deliver second pixel
                wrist.setPosition(dWristDeliver);
                InsidePixel.setPosition(dInsideHold);
                OutsidePixel.setPosition(dOutside2Pixel);
        }
    }

    private void Intake(){
        Intake.setPower(dIntakeSpeed);
        intake_servo_1.setPower(dIntakeSpeed);
        intake_servo_2.setPower(dIntakeSpeed);
    }

    private void ScissorLift(){
        ScissorLeft.setPower(LeftLiftPower);
        ScissorRight.setPower(RightLiftPower);
    }

    private void P6(){
        p6servo.setPosition(dP6ServoPosition);
    }

    private void Drone(){
        drone.setPosition(dDronePos);
    }

    private void Drive (){
        leftfront_drive.setPower(dLFDrivePower);
        rightfront_drive.setPower(dRFDrivePower);
        rightback_drive.setPower(dRBDrivePower);
        leftback_drive.setPower(dLBDrivePower);
    }
}