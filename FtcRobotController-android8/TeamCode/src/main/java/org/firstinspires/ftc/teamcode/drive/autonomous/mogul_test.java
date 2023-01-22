package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Queue;

@Autonomous(group = "drive")
public class mogul_test extends LinearOpMode {
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

//        DcMotorEx SlideMotor;
//        Servo ClawServo;
//        Servo ArmServo;
//        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
//        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
//        ArmServo = hardwareMap.get(Servo.class, "ArmServo");

//        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
        telemetry.update();

        int vision = 1;

//        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        SlideMotor.setTargetPosition(0);
//        ClawServo.setPosition(0.62);
//        ClawServo.setDirection(Servo.Direction.REVERSE);
//        ArmServo.setPosition(0);
//        SlideMotor.setVelocity(10000);

//        Pose2d startPose = new Pose2d(-60, -7, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .forward(40)
//                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(45)))
//                .waitSeconds(2)
//                .lineToSplineHeading(new Pose2d(-8, -32, Math.toRadians(90)))
//                .waitSeconds(3)
//                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(45)))
//                .waitSeconds(2)
//                .lineToSplineHeading(new Pose2d(-8, -32, Math.toRadians(90)))
//                .waitSeconds(3)
//                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(45)))
//                .waitSeconds(2)
//                .lineToSplineHeading(new Pose2d(-8, -32, Math.toRadians(90)))
//                .build();
//
//        waitForStart();
//
//        telemetry.addData("location is", startPose.toString());
//
//        if (!isStopRequested())
//            drive.followTrajectorySequence(trajSeq);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(39)
                .lineToSplineHeading(new Pose2d(60, 7, Math.toRadians(45)))
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(52, -26, Math.toRadians(90)))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(58.5, 8, Math.toRadians(45)))
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(51, -26, Math.toRadians(90)))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(58.5, 8, Math.toRadians(45)))
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(50, -27, Math.toRadians(90)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);

//        Trajectory fromStart = drive.trajectoryBuilder(startPose)
//                .forward(40)
//                .build();
//
//        Trajectory moveToGoal = drive.trajectoryBuilder(fromStart.end())
//                .lineToSplineHeading(new Pose2d(12, 5, Math.toRadians(45)))
//                .build();



//        Trajectory scoreFirstCone = drive.trajectoryBuilder(start.end())
//                .forward(48)
//                .splineToConstantHeading(new Vector2d(54,10),Math.toRadians(45))
//                .build();
//
//        Trajectory moveForwardAnInch = drive.trajectoryBuilder(scoreFirstCone.end())
//                .forward(3)
//                .build();
//
//        Trajectory pickupNewCone = drive.trajectoryBuilder(moveForwardAnInch.end())
//                .back(6)
//                .addDisplacementMarker(() ->{
//                    ArmServo.setPosition(0.68);
//                })
//                .splineTo(new Vector2d(48,-22),Math.toRadians(-90))
//                .build();
//
//        Trajectory returnToPole = drive.trajectoryBuilder(pickupNewCone.end())
//                .splineTo(new Vector2d(52,8),Math.toRadians(45))
//                .build();
//
//        Trajectory returnToPolePt2 = drive.trajectoryBuilder(returnToPole.end())
//                .forward(6)
//                .build();
//
//        Trajectory returnToPolePt3 = drive.trajectoryBuilder(returnToPolePt2.end())
//                .strafeRight(2)
//                .build();
//
//        Trajectory pickupAnotherCone = drive.trajectoryBuilder(returnToPolePt3.end())
//                .back(6)
//                .addDisplacementMarker(() ->{
//                    ArmServo.setPosition(0.68);
//                })
//                .splineTo(new Vector2d(52,-24),Math.toRadians(-90))
//                .build();
//
//        Trajectory returnToPoleAgain = drive.trajectoryBuilder(pickupAnotherCone.end())
//                .splineTo(new Vector2d(52,8),Math.toRadians(45))
//                .build();
//
//        Trajectory returnToPoleAgainPt2 = drive.trajectoryBuilder(returnToPoleAgain.end())
//                .forward(6)
//                .build();
//
//        Trajectory returnToPoleAgainPt3 = drive.trajectoryBuilder(returnToPoleAgainPt2.end())
//                .strafeRight(5)
//                .build();
//
//        Trajectory scoreLeft = drive.trajectoryBuilder(returnToPoleAgainPt3.end())
//                .back(4)
//                .build();
//
//        Trajectory scoreLeftPt2 = drive.trajectoryBuilder(returnToPoleAgainPt3.end())
//                .lineTo(new Vector2d(54,26))
//                .build();
//
//        Trajectory scoreMiddle = drive.trajectoryBuilder(returnToPoleAgainPt3.end())
//                .back(4)
//                .build();
//
//        Trajectory scoreRight = drive.trajectoryBuilder(returnToPoleAgainPt3.end())
//                .back(4)
//                .build();
//
//        Trajectory scoreRightPt2 = drive.trajectoryBuilder(returnToPoleAgainPt3.end())
//                .lineTo(new Vector2d(54,-22))
//                .build();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
//        /* Actually do something useful */
//        if(tagOfInterest == null){
//            vision = 2;
//        }else if(tagOfInterest.id == LEFT){
//            vision = 1;
//        }else if(tagOfInterest.id == MIDDLE){
//            vision = 2;
//        }else{
//            vision = 3;
//        }



//        SlideMotor.setTargetPosition(4200);
//        drive.followTrajectory((fromStart));
//        drive.followTrajectory(scoreFirstCone);
//        drive.turn(Math.toRadians(45));
//        drive.followTrajectory(moveForwardAnInch);
//        SlideMotor.setTargetPosition(3700);
//        sleep(500);
//        ClawServo.setPosition(0.85);
//        SlideMotor.setTargetPosition(600);
//        drive.followTrajectory(pickupNewCone);
//        ClawServo.setPosition(0.62);
//        sleep(500);
//        SlideMotor.setTargetPosition(1200);
//        sleep(500);
//        ArmServo.setPosition(0);
//        sleep(500);
//        SlideMotor.setTargetPosition(4200);
//        drive.followTrajectory(returnToPole);
//        drive.followTrajectory(returnToPolePt2);
//        drive.followTrajectory(returnToPolePt3);
//        SlideMotor.setTargetPosition(3700);
//        sleep(500);
//        ClawServo.setPosition(0.85);
//        SlideMotor.setTargetPosition(400);
//        drive.followTrajectory(pickupAnotherCone);
//        ClawServo.setPosition(0.62);
//        sleep(500);
//        SlideMotor.setTargetPosition(1200);
//        sleep(500);
//        ArmServo.setPosition(0);
//        sleep(500);
//        SlideMotor.setTargetPosition(4200);
//        drive.followTrajectory(returnToPoleAgain);
//        drive.followTrajectory(returnToPoleAgainPt2);
//        drive.followTrajectory(returnToPoleAgainPt3);
//        SlideMotor.setTargetPosition(3700);
//        sleep(500);
//        ClawServo.setPosition(0.85);
//        if(vision == 1){
//            drive.followTrajectory(scoreLeft);
//            drive.followTrajectory(scoreLeftPt2);
//        } else if (vision == 2){
//            drive.followTrajectory(scoreMiddle);
//        } else if (vision == 3){
//            drive.followTrajectory(scoreRight);
//            drive.followTrajectory(scoreRightPt2);
//        }
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

