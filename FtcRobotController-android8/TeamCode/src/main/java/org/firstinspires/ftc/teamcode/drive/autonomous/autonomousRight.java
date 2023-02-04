package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.annotation.SuppressLint;
import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.autonomous.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "drive")
@Disabled
public class autonomousRight extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1394.6027293299926;
    double fy = 1394.6027293299926;
    double cx = 995.588675691456;
    double cy = 599.3212928484164;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx SlideMotor;
        Servo ClawServo;
        Servo ArmServo;
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");

        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
        telemetry.update();

        int vision = 1;

        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideMotor.setTargetPosition(0);
        ClawServo.setPosition(0.62);
        ClawServo.setDirection(Servo.Direction.REVERSE);
        ArmServo.setPosition(0);
        SlideMotor.setVelocity(10000);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(54)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(56,18,-45))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(2)
                .build();

        Trajectory traj44 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(3)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj44.end())
                .back(15)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(49, -29, 80))
                .build();

        Trajectory traj66 = drive.trajectoryBuilder(traj6.end())
                .lineToLinearHeading(traj5.end())
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj66.end())
                .lineToLinearHeading(new Pose2d(54,20,0))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d(56,18,-45))
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .forward(2)
                .build();

        Trajectory traj99 = drive.trajectoryBuilder(traj9.end())
                .strafeLeft(3)
                .build();

        Trajectory traj11 = drive.trajectoryBuilder(traj99.end())
                .back(10)
                .build();

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(49, -29, 0))
                .build();

        Trajectory traj13 = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(49, -5, 0))
                .build();

        Trajectory traj14 = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(49, 29, 0))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            vision = 1;
        }else if(tagOfInterest.id == LEFT){
            vision = 3;
        }else if(tagOfInterest.id == MIDDLE){
            vision = 2;
        }else{
            vision = 1;
        }

        SlideMotor.setPower(0);
        drive.followTrajectory(traj1);
        SlideMotor.setTargetPosition(4100);
        SlideMotor.setPower(1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        do {
        } while (SlideMotor.isBusy());
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj44);
        SlideMotor.setTargetPosition(3700);
        ClawServo.setPosition(0.85);
        sleep(1000);
        drive.followTrajectory(traj5);
        sleep(500);
        ClawServo.setPosition(0.85);
        SlideMotor.setTargetPosition(600);
        drive.followTrajectory(traj6);
        ClawServo.setPosition(0.62);
        sleep(500);
        SlideMotor.setTargetPosition(1500);
        sleep(500);
        drive.followTrajectory(traj66);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        SlideMotor.setTargetPosition(4100);
        SlideMotor.setPower(1);
        do {
        } while (SlideMotor.isBusy());
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj99);
        SlideMotor.setTargetPosition(3500);
        sleep(500);
        ClawServo.setPosition(0.85);
        sleep(500);
        SlideMotor.setTargetPosition(0);
        drive.followTrajectory(traj11);
        ArmServo.setPosition(0.68);
        sleep(500);
        drive.turn(Math.toRadians(-45));
        if(vision == 1){
            drive.followTrajectory(traj12);
        } else if (vision == 2){
            drive.followTrajectory(traj13);
        } else if (vision == 3){
            drive.followTrajectory(traj14);
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

