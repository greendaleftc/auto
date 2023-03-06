package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;
import android.text.method.Touch;
import android.transition.Slide;
import android.widget.Button;

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
        boolean touch_sensor_hit = false;
        boolean yb_position = false;
        boolean high_med_position = false;


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

        int hit_count = 0;

        double block_arm_pos_val = .82;


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
                    BlockServo.setPosition(block_arm_pos_val);
                    sleep(750);
                    ArmServo.setPosition(0);
                    high_med_position = true;
                    low_position = false;
                    yb_position = false;

                } else if
                (gamepad1.dpad_left) {
                    //Low
                    SlideMotor.setTargetPosition(1700);
                    ((DcMotorEx) SlideMotor).setVelocity(20000);
                    BlockServo.setPosition(0);
                    sleep(750);
                    ArmServo.setPosition(0);
                    high_med_position = false;
                    low_position = true;
                    yb_position = false;

                } else if (gamepad1.dpad_up) {
                    if (yb_position) {
                        SlideMotor.setTargetPosition(3900);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        BlockServo.setPosition(block_arm_pos_val);
                        sleep(750);
                        ArmServo.setPosition(0);
                        yb_position = false;
                        high_med_position = true;

                    }else {

                        //High
                        SlideMotor.setTargetPosition(3900);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        BlockServo.setPosition(block_arm_pos_val);
                        sleep(750);
                        ArmServo.setPosition(0);
                        low_position = false;
                        yb_position = false;
                        high_med_position = true;
                    }
                    //Claw to Ground
                } else if (gamepad1.dpad_down) {
                    if (yb_position && ArmServo.getPosition() == 0) {
                        SlideMotor.setTargetPosition(0);
                        ArmServo.setPosition(0);
                        yb_position = false;
                        high_med_position = false;
                    }else if (yb_position && ArmServo.getPosition() == .68) {
                        ArmServo.setPosition(.68);
                        sleep(750);
                        SlideMotor.setTargetPosition(0);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        low_position = false;
                        high_med_position = false;
                    }else if (low_position) {
                        ArmServo.setPosition(0);
                        sleep(750);
                        SlideMotor.setTargetPosition(0);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        low_position = false;
                        high_med_position = false;
                    } else {
                        SlideMotor.setTargetPosition(0);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        ArmServo.setPosition(.68);
                        low_position = false;
                        high_med_position = false;
                    }

                    //pole guide
                } else if (gamepad1.left_stick_button) {
                    BlockServo.setPosition(block_arm_pos_val);
                } else if (gamepad1.right_stick_button) {
                    BlockServo.setPosition(0);
                }

            // B height
                else if (gamepad1.b) {
                    if (high_med_position) {
                        BlockServo.setPosition(0);
                        SlideMotor.setTargetPosition(500);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        yb_position = true;
                        high_med_position = false;
                    } else {
                        SlideMotor.setTargetPosition(500);
                        ((DcMotorEx) SlideMotor).setVelocity(20000);
                        yb_position = true;
                    }
                }
                // Y height
                else if (gamepad1.y) {
                        if (high_med_position) {
                            BlockServo.setPosition(0);
                            SlideMotor.setTargetPosition(300);
                            ((DcMotorEx) SlideMotor).setVelocity(20000);
                            high_med_position = false;
                            yb_position = true;
                        } else {
                            SlideMotor.setTargetPosition(300);
                            ((DcMotorEx) SlideMotor).setVelocity(20000);
                            yb_position = true;
                        }
                        }
                        //open claw
                        if (gamepad1.right_trigger == 0) {
                            ClawServo.setPosition(0.85);
                        }
                        //close claw
                        if (gamepad1.right_trigger == 1) {
                            ClawServo.setPosition(0.63);
                        }

                        //Emergency zeroize slide
//                if (gamepad1.back) {
//                    while (!TouchSensor.isPressed()){
//                        SlideMotor.setPower(-.5);
////                        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                        SlideMotor.setTargetPosition(0);
//                    break;}
//                }


                        //At zero
                        if (TouchSensor.getValue() == 1) {

                            SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SlideMotor.setTargetPosition(0);

                            hit_count++;

                        }

                        drive.update();

                        Pose2d poseEstimate = drive.getPoseEstimate();
                        telemetry.addData("x", poseEstimate.getX());
                        telemetry.addData("y", poseEstimate.getY());
                        telemetry.addData("heading", poseEstimate.getHeading());
                        telemetry.addData("slide height", slide_height);
                        telemetry.addData("slide button pressed", TouchSensor.isPressed());
                        telemetry.addData("low position bool", low_position);
                        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
                        telemetry.addData("Claw Pos", ClawServo.getPosition());
                        telemetry.addData("Reset Count", hit_count);
                        telemetry.addData("slide info", SlideMotor.getConnectionInfo());
                        telemetry.addData("button value", TouchSensor.getValue());
                        telemetry.addData("yb hight", yb_position);
                        telemetry.addData("High-Med height", high_med_position);
                        telemetry.update();



        }
    }
}
