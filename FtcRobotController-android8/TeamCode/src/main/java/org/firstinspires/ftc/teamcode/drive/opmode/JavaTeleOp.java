package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(group = "drive")
public class JavaTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx rightFront;
        DcMotorEx rightRear;
        DcMotorEx leftFront;
        DcMotorEx leftRear;
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");

        Servo ClawServo;
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");

        Servo ArmServo;
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");

        Servo BlockServo;
        BlockServo = hardwareMap.get(Servo.class, "BlockServo");

        TouchSensor TouchSensor;
        TouchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        DcMotorEx SlideMotor;
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");

        boolean low_position = false;
        boolean yb_position = false;
        boolean high_med_position = false;
        int hit_count = 0;
        double block_arm_pos_val = .82;
        double safety_speed = 0.65;
        float vertical;
        float horizontal;
        double pivot;

        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
        telemetry.update();

        //waiting for start button press
        waitForStart();

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideMotor.setTargetPosition(0);
        SlideMotor.setVelocity(10000);

        ClawServo.setDirection(Servo.Direction.REVERSE);

        BlockServo.setPosition(0);

        while (!isStopRequested()) {

//            ClawServo.setPosition(.5);
            int slide_height = SlideMotor.getCurrentPosition();



//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x,
//                            -gamepad1.right_stick_x
//
//                    )
//            );

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

                vertical = gamepad1.left_stick_y;
                horizontal = -gamepad1.left_stick_x;
                pivot = 0.8 * -gamepad1.right_stick_x;
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                rightFront.setPower((-pivot + (vertical - horizontal)) * safety_speed);
                rightRear.setPower((-pivot + vertical + horizontal) * safety_speed);
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                leftFront.setPower((pivot + vertical + horizontal) * safety_speed);
                leftRear.setPower((pivot + (vertical - horizontal)) * safety_speed);

                        //drive.update();

                        //Pose2d poseEstimate = drive.getPoseEstimate();
//                        telemetry.addData("x", poseEstimate.getX());
//                        telemetry.addData("y", poseEstimate.getY());
//                        telemetry.addData("heading", poseEstimate.getHeading());
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
