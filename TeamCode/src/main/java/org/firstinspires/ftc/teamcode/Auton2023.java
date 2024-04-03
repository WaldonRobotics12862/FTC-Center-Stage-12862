package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auton2023")
public class Auton2023 extends LinearOpMode {
    WaldonHardware robot = new WaldonHardware(this);
    Variables variables = new Variables(this);
    int rightCenterLeft = 0;
    double Left_Distance = 2000;
    double Right_Distance = 2000;
    double Center_Distance = 2000;
    NormalizedRGBA CurrentColor;


    private void turnLeft() {
        drive(0, 0, 22, 0.3);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void turnRight() {
        drive(0, 0, -22, 0.3);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void p6() {
        robot.p6servo.setPosition(0.5);
    }

    private void callStop() {
        if (isStopRequested()) {
            terminateOpModeNow();
        }
    }

    private void drive(int forward, int strafe, int turn, double speed) {
        // Forward/Backwards-Positve values go forward, Negative values go backwards
        // Turn Left/Right Positve values turn left, Negative values will turn right, Power at 0.3 and Turn at 22 is very close to 90 degree turn
        // Need to fine tune the turning values and make sure it is good
        // Strafe Left/Right Positve values strafe left, Negative values strafe right, check this
        robot.leftback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftfront_drive.setTargetPosition((int) ((forward * -1 + strafe + turn * -1) * variables.countsPerInch));
        robot.leftback_drive.setTargetPosition((int) ((forward * -1 - (strafe - turn * -1)) * variables.countsPerInch));
        robot.rightfront_drive.setTargetPosition((int) ((forward * -1 - (strafe + turn * -1)) * variables.countsPerInch));
        robot.rightback_drive.setTargetPosition((int) ((forward * -1 + (strafe - turn * -1)) * variables.countsPerInch));
        robot.leftback_drive.setPower(speed * 1);
        robot.leftfront_drive.setPower(speed * 1);
        robot.rightback_drive.setPower(speed * 1);
        robot.rightfront_drive.setPower(speed * 1);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.rightback_drive.isBusy() && robot.rightfront_drive.isBusy() && robot.leftback_drive.isBusy() && robot.leftfront_drive.isBusy()) {
            callStop();
        }
        sleep(450);
        robot.leftback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private void DecideSpikeMark() {
        Left_Distance = 2000;
        Right_Distance = 2000;
        Center_Distance = 2000;

        if (robot.leftDistanceSensor.getDistance(DistanceUnit.INCH) < Left_Distance) {
            Left_Distance = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        }
        if (robot.centerDistanceSensor.getDistance(DistanceUnit.INCH) < Center_Distance) {
            Center_Distance = robot.centerDistanceSensor.getDistance(DistanceUnit.INCH);
        }
        if (robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) < Right_Distance) {
            Right_Distance = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        }
        telemetry.addData("Left", Left_Distance);
        telemetry.addData("Center", Center_Distance);
        telemetry.addData("Right", Right_Distance);
        telemetry.update();
        if (Right_Distance < 6) {
            rightCenterLeft = 3;
        } else if (Left_Distance < 6) {
            rightCenterLeft = 1;
        } else {
            rightCenterLeft = 2;
        }
    }

    private void StrafeRightToSpikeMark() {
        robot.leftback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setPower(-0.3);
        robot.leftback_drive.setPower(0.3);
        robot.rightback_drive.setPower(-0.3);
        robot.rightfront_drive.setPower(0.3);
        while (opModeIsActive()) {
            CurrentColor = ((NormalizedColorSensor) robot.p6Color).getNormalizedColors();
            telemetry.addData("Blue", robot.p6Color.blue());
            telemetry.addData("Red", robot.p6Color.red());
            telemetry.update();
            if (robot.p6Color.blue() >= Variables.blue_threshold || robot.p6Color.red() >= Variables.red_threshold) {
                robot.leftback_drive.setPower(0);
                robot.leftfront_drive.setPower(0);
                robot.rightback_drive.setPower(0);
                robot.rightfront_drive.setPower(0);
                break;
            }
        }
    }

    private void DriveToWall() {
        robot.leftback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftback_drive.setPower(-0.2);
        robot.leftfront_drive.setPower(-0.2);
        robot.rightback_drive.setPower(-0.2);
        robot.rightfront_drive.setPower(-0.2);
        Center_Distance = robot.centerDistanceSensor.getDistance(DistanceUnit.CM);
        while (Center_Distance > 10) {
            Center_Distance = robot.centerDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Center", Center_Distance);
            telemetry.update();
            if (Center_Distance < 10) {
                robot.leftback_drive.setPower(0);
                robot.leftfront_drive.setPower(0);
                robot.rightback_drive.setPower(0);
                robot.rightfront_drive.setPower(0);
                break;
            }
        }
    }

    private void DriveToSpike() {
        robot.leftback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightback_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightfront_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightback_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightfront_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftback_drive.setPower(-0.2);
        robot.leftfront_drive.setPower(-0.2);
        robot.rightback_drive.setPower(-0.2);
        robot.rightfront_drive.setPower(-0.2);
        while (opModeIsActive()) {
            CurrentColor = ((NormalizedColorSensor) robot.p6Color).getNormalizedColors();
            telemetry.addData("Blue", robot.p6Color.blue());
            telemetry.addData("Red", robot.p6Color.red());
            telemetry.update();
            if (robot.p6Color.blue() >= Variables.blue_threshold || robot.p6Color.red() >= Variables.red_threshold) {
                robot.leftback_drive.setPower(0);
                robot.leftfront_drive.setPower(0);
                robot.rightback_drive.setPower(0);
                robot.rightfront_drive.setPower(0);
                robot.leftback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightback_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightfront_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            }
        }
    }
    @Override
    public void runOpMode() {
        robot.initialize();

        ElapsedTime Timer;
        Timer = new ElapsedTime();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        drive(26, 0, 0, 0.5);
        DriveToSpike();
        drive(-2, 0, 0, 0.2);
        DecideSpikeMark();
        if (rightCenterLeft == 1) {
            // Left Spike Mark
            drive(-5, 0, 0, 0.3);
            drive(0, -5, 0, 0.3);
            turnLeft();
            DriveToSpike();
            sleep(100);
            p6();
            sleep(1000);
            drive(-15, 0, 0, 0.6);
            turnRight();
            drive(-32, 0, 0, 0.6);
            drive(0, -47, 0, 0.6);
            robot.wrist.setPosition(Variables.wristDelivery);
        } else if (rightCenterLeft == 3) {
            // Right Spike Mark
            StrafeRightToSpikeMark();
            sleep(100);
            p6();
            sleep(1000);
            drive(-30, 0, 0, 0.5);
            drive(0, -60, 0, 0.5);
            robot.wrist.setPosition(Variables.wristDelivery);
        } else {
            // Center Spike Mark
            DriveToSpike();
            sleep(100);
            p6();
            sleep(1000);
            drive(-11, 0, 0, 0.6);
            drive(0, -30, 0, 0.6);
            drive(7, 0, 0, 0.6);
            turnRight();
            robot.wrist.setPosition(Variables.wristDelivery);
            DriveToWall();
            robot.InsidePixel.setPosition(Variables.InsidePixelSingle);
            robot.OutsidePixel.setPosition(Variables.OutsidePixelSingle);
        }
    }
}
