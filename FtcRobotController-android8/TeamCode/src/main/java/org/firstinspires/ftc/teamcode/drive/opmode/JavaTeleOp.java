package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;
import android.text.method.Touch;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import java.util.ArrayList;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
public class JavaTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx SlideMotor;
        Servo ClawServo;
        Servo ArmServo;
        Servo BlockServo;
        TouchSensor TouchSensor;
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");
        BlockServo = hardwareMap.get(Servo.class, "BlockServo");
        TouchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");
        boolean low_position = false;

        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideMotor.setTargetPosition(0);
        ClawServo.setDirection(Servo.Direction.REVERSE);

        SlideMotor.setVelocity(10000);
        BlockServo.setPosition(0);

        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {

//            ClawServo.setPosition(.5);
            int slide_height = SlideMotor.getCurrentPosition();



            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x

                    )
            );

                // linear slide code
                    // Mid
                if (gamepad1.dpad_right) {
                    SlideMotor.setTargetPosition(2900);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);
                    BlockServo.setPosition(0.85);
                    sleep(750);
                    ArmServo.setPosition(0);
                    low_position = false;

                } else if
                (gamepad1.dpad_left) {
                    //Low
                    SlideMotor.setTargetPosition(1700);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);
                    BlockServo.setPosition(0.85);
                    sleep(750);
                    ArmServo.setPosition(0);
                    low_position = true;

                } else if (gamepad1.dpad_up) {
                    //High
                    SlideMotor.setTargetPosition(3900);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);
                    BlockServo.setPosition(0.85);
                    sleep(750);
                    ArmServo.setPosition(0);
                    low_position = false;

                } else if (gamepad1.b) {
                    SlideMotor.setTargetPosition(500);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);

                } else if (gamepad1.y) {
                    SlideMotor.setTargetPosition(300);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);


                }
                //Claw to Ground
                else if (gamepad1.dpad_down) {
                    if (low_position == true) {
                        ArmServo.setPosition(.68);
                        sleep(750);
                        SlideMotor.setTargetPosition(0);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                    } else {
                        SlideMotor.setTargetPosition(0);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        ArmServo.setPosition(.68);
                    }


                //pole guide
                } else if (gamepad1.left_stick_button) {
                    BlockServo.setPosition(0.85);
                } else if (gamepad1.right_stick_button) {
                    BlockServo.setPosition(0);
                }

                    //open claw
                if (gamepad1.right_trigger == 0) {
                    ClawServo.setPosition(0.85);
                }
                    //close claw
                if (gamepad1.right_trigger == 1) {
                    ClawServo.setPosition(0.63);
                }

                //Button Press to Reset
                if (TouchSensor.isPressed()) {
                    SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SlideMotor.setTargetPosition(0);
                }

                drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("slide height", slide_height);
            telemetry.addData("reset slide button", TouchSensor.isPressed());
            telemetry.addData("low position bool", low_position);
            telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
            telemetry.addData("Claw Pos", ClawServo.getPosition());
            telemetry.update();



        }
    }
}
