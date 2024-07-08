package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Teleop2023")
public class Teleop2023 extends LinearOpMode {

    WaldonHardware robot = new WaldonHardware(this);
    Variables variables = new Variables(this);

    @Override
    public void runOpMode() {
         robot.initialize();
         init_IMU();

        // Wait for the match to begin.
        waitForStart();
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

            telemetry.addData("Strafe", variables.dStrafe);
            telemetry.addData("Forward", variables.dForward);
            telemetry.addData("Turn", variables.dTurn);
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
        variables.imuParameters = new BNO055IMU.Parameters();
        variables.imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        variables.imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        variables.imuParameters.mode = BNO055IMU.SensorMode.IMU;

        robot.imu.initialize(variables.imuParameters);
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
        //if (gamepad1.left_bumper) {
        //    variables.dDriveScale = 0.68;
        //}
        //if (gamepad1.right_bumper) {
        //    variables.dDriveScale = 0.5;
        //}
        // Calculate the drive parameters
        variables.dTurn = (-gamepad1.right_stick_x * variables.dDriveScale); // adding the (double) casts this as a double variable to match the dTurn.  Not sure if the right_stick_x is a double by default or not)
        variables.dDrive = Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), 0, 1);  // dDrive = sqrt(Y^2 + X^2). the clipping function clips any values to be just between 0 and 1
        variables.dGamePadDegree = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180; // calculate the angle by use of the arc tangent
        variables.dMovement = variables.dGamePadDegree - dGetZAxis();
        variables.dStrafe = Math.cos(variables.dMovement / 180 * Math.PI) * variables.dDrive * variables.dDriveScale;
        variables.dForward = Math.sin(variables.dMovement / 180 * Math.PI) * variables.dDrive * variables.dDriveScale;

        variables.dDenominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(variables.dForward) + Math.abs(variables.dStrafe + Math.abs(variables.dTurn)), 1));
        variables.dLFDrivePower = ((variables.dForward * Math.abs(variables.dForward) + variables.dStrafe * Math.abs(variables.dStrafe)) + variables.dTurn) / variables.dDenominator;
        variables.dRFDrivePower = ((variables.dForward * Math.abs(variables.dForward) - variables.dStrafe * Math.abs(variables.dStrafe)) - variables.dTurn) / variables.dDenominator;
        variables.dRBDrivePower = ((variables.dForward * Math.abs(variables.dForward) + variables.dStrafe * Math.abs(variables.dStrafe)) - variables.dTurn) / variables.dDenominator;
        variables.dLBDrivePower = ((variables.dForward * Math.abs(variables.dForward) - variables.dStrafe * Math.abs(variables.dStrafe)) + variables.dTurn) / variables.dDenominator;
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
            variables.iDeliveryState = 0;
        }
        if (gamepad2.b){
            variables.iDeliveryState = 1;
        }
        if (gamepad2.x){
            if (variables.iDeliveryState == 2){
                variables.iDeliveryState = 3;
            } else {
                variables.iDeliveryState = 2;
            }
        }
        if (gamepad2.guide){
            robot.dDronePos = 0.5;
        }
        if (gamepad2.dpad_down) {
            //set both lift powers to -1
            variables.LeftLiftPower = -1;
            variables.RightLiftPower = -1;
        } else if (gamepad2.dpad_up) {
            // set both lift powers to 1
            variables.LeftLiftPower = 1;
            variables.RightLiftPower = 1;
        } else {
            // set both lift powers to the trigger value so that we can gracefully lower them
            variables.LeftLiftPower = -gamepad2.left_trigger;
            variables.RightLiftPower = -gamepad2.right_trigger;
        }

        if (gamepad2.left_bumper){
            variables.dIntakeSpeed = 1;
        } else if (gamepad2.right_bumper){
            variables.dIntakeSpeed = -1;
        } else {
            variables.dIntakeSpeed = 0;
        }
    }

    private void Deliver_Mechanism(){
        switch(variables.iDeliveryState) {
            case 0: //reset
                robot.wrist.setPosition(variables.dWristIn);
                robot.InsidePixel.setPosition(variables.dInsideIn);
                robot.OutsidePixel.setPosition(variables.dOutsideIn);
                break;
            case 1: //Deliver setup
                robot.wrist.setPosition(variables.dWristDeliver);
                robot.InsidePixel.setPosition(variables.dInsideHold);
                robot.OutsidePixel.setPosition(variables.dOutsideIn);
                break;
            case 2: //Deliver pixel
                robot.wrist.setPosition(variables.dWristDeliver);
                robot.InsidePixel.setPosition(variables.dInsideHold);
                robot.OutsidePixel.setPosition(variables.dOutside1Pixel);
                break;
            case 3: // Deliver second pixel
                robot.wrist.setPosition(variables.dWristDeliver);
                robot.InsidePixel.setPosition(variables.dInsideHold);
                robot.OutsidePixel.setPosition(variables.dOutside2Pixel);
        }
    }

    private void Intake(){
        robot.Intake.setPower(variables.dIntakeSpeed);
        robot.intake_servo_1.setPower(variables.dIntakeSpeed);
        robot.intake_servo_2.setPower(variables.dIntakeSpeed);
    }

    private void ScissorLift(){
        robot.ScissorLeft.setPower(variables.LeftLiftPower);
        robot.ScissorRight.setPower(variables.RightLiftPower);
    }

    private void P6(){
        robot.p6servo.setPosition(variables.p6servo);
    }

    private void Drone(){
        robot.drone.setPosition(variables.dDronePos);
    }

    private void Drive (){
        robot.leftfront_drive.setPower(variables.dLFDrivePower);
        robot.rightfront_drive.setPower(variables.dRFDrivePower);
        robot.leftback_drive.setPower(variables.dLBDrivePower);
        robot.rightback_drive.setPower(variables.dRBDrivePower);
    }
}