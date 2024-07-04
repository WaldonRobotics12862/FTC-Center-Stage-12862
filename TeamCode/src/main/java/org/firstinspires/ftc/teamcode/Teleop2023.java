package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Teleop2023")
public class Teleop2023 extends LinearOpMode {

    WaldonHardware robot = new WaldonHardware(this);
//    Variables variables = new Variables(this);

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

    //Lift Varables
    double LeftLiftPower = 0;
    double RightLiftPower = 0;

    int iDeliveryState = 0;
    BNO055IMU.Parameters imuParameters;  // imuParameters is a container, so we'll have to initalize that upon startup
    public double dDenominator  = 1; // don't set a denominator to 0 or we'll get a divide by 0 error

    @Override
    public void runOpMode() {
         robot.initialize();
         // Wait for the match to begin.
        waitForStart();
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

            telemetry.addData("Strafe", dStrafe);
            telemetry.addData("Forward", dForward);
            telemetry.addData("Turn", dTurn);
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
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

        robot.imu.initialize(imuParameters);
    }

    private double dGetZAxis() {
        //check which angle we are at according to the IMU.
        Orientation oAngle;
        oAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
            robot.dP6ServoPosition = 0.5;
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
            robot.dDronePos = 0.5;
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
            robot.dIntakeSpeed = 1;
        } else if (gamepad2.right_bumper){
            robot.dIntakeSpeed = -1;
        } else {
            robot.dIntakeSpeed = 0;
        }
    }

    private void Deliver_Mechanism(){
        switch(iDeliveryState) {
            case 0: //reset
                robot.wrist.setPosition(Variables.dWristIn);
                robot.InsidePixel.setPosition(Variables.dInsideIn);
                robot.OutsidePixel.setPosition(Variables.dOutsideIn);
                break;
            case 1: //Deliver setup
                robot.wrist.setPosition(Variables.dWristDeliver);
                robot.InsidePixel.setPosition(Variables.dInsideHold);
                robot.OutsidePixel.setPosition(Variables.dOutsideIn);
                break;
            case 2: //Deliver pixel
                robot.wrist.setPosition(Variables.dWristDeliver);
                robot.InsidePixel.setPosition(Variables.dInsideHold);
                robot.OutsidePixel.setPosition(Variables.dOutside1Pixel);
                break;
            case 3: // Deliver second pixel
                robot.wrist.setPosition(Variables.dWristDeliver);
                robot.InsidePixel.setPosition(Variables.dInsideHold);
                robot.OutsidePixel.setPosition(Variables.dOutside2Pixel);
        }
    }

    private void Intake(){
        robot.Intake.setPower(Variables.dIntakeSpeed);
        robot.intake_servo_1.setPower(Variables.dIntakeSpeed);
        robot.intake_servo_2.setPower(Variables.dIntakeSpeed);
    }

    private void ScissorLift(){
        robot.ScissorLeft.setPower(LeftLiftPower);
        robot.ScissorRight.setPower(RightLiftPower);
    }

    private void P6(){
        robot.p6servo.setPosition(Variables.p6servo);
    }

    private void Drone(){
        robot.drone.setPosition(Variables.dDronePos);
    }

    private void Drive (){
        robot.leftfront_drive.setPower(dLFDrivePower);
        robot.rightfront_drive.setPower(dRFDrivePower);
        robot.rightback_drive.setPower(dRBDrivePower);
        robot.leftback_drive.setPower(dLBDrivePower);
    }
}