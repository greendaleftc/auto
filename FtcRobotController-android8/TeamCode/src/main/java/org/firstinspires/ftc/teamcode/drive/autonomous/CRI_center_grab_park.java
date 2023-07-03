package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class CRI_center_grab_park extends LinearOpMode {
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

    ////////// FIELD LOCATIONS

    // high goal
    double highGoal_x_first = 81;
    double highGoal_y_first = -6;

    // medium goal
    double medGoal_x = 39.5;
    double medGoal_y = 8.5;

    // cone stack
    double coneStack_x = 49.75;
    double coneStack_y = -22.75;

    // cone stack last 2
    //double coneStack_x_return = 50.5;
    //double coneStack_y_return = -22.5;

    // height 1st cone from stack
    int cone1_height = 580;

    // height 2nd cone from stack
    int cone2_height = 450;

    // height 3rd cone from stack
    int cone3_hight = 325;

    // height 4th cone from stack
    int cone4_height = 200;

    // medium goal last 2
    //double medGoal_x_last2 = 37.75;
    //double medGoal_y_last2 = 8.25;

    // return high goal
    //double highGoal_x_return = 57.6;
    //double highGoal_y_return = 8.1;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx SlideMotor;
        Servo ClawServo;
        Servo ArmServo;
        Servo BlockServo;
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        ArmServo = hardwareMap.get(Servo.class, "ArmServo");
        BlockServo = hardwareMap.get(Servo.class, "BlockServo");

        telemetry.addData("Slide Pos", SlideMotor.getCurrentPosition());
        telemetry.update();

        int vision = 1;

        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideMotor.setTargetPosition(0);
        ClawServo.setPosition(0.64);
        ClawServo.setDirection(Servo.Direction.REVERSE);
        ArmServo.setPosition(0);
        SlideMotor.setVelocity(10000);
        ClawServo.setPosition(0.64);
        BlockServo.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
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


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
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
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            vision = 2;
        } else if (tagOfInterest.id == LEFT) {
            vision = 1;
        } else if (tagOfInterest.id == MIDDLE) {
            vision = 2;
        } else {
            vision = 3;
        }
         {


             if (vision == 1) {
                 Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

                 drive.setPoseEstimate(startPose);

                 TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                         .addTemporalMarker(.7, () -> SlideMotor.setTargetPosition(3900))
                         .addTemporalMarker(1.2, () -> BlockServo.setPosition(0.82))
                         .lineTo(new Vector2d(73, 0))
                         .splineTo(new Vector2d(highGoal_x_first, highGoal_y_first), Math.toRadians(-45)) // new fluid movement
                         .waitSeconds(.1)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                         .back(8)
                         .UNSTABLE_addTemporalMarkerOffset(.0, () -> BlockServo.setPosition(0))
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .lineToSplineHeading(new Pose2d(74, -50, Math.toRadians(-45)))
                         .lineToSplineHeading(new Pose2d(89, -72, Math.toRadians(270)))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.63))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(300))
                         .back(10)
                         .strafeRight(15)
                         .lineToLinearHeading(new Pose2d(74, 0, Math.toRadians(0)))
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .build();

                 if (!isStopRequested())
                     drive.followTrajectorySequence(trajSeq);

             } else if (vision == 2) {
                 Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

                 drive.setPoseEstimate(startPose);

                 TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                         .addTemporalMarker(0.7, () -> SlideMotor.setTargetPosition(3900))
                         .addTemporalMarker(1.2, () -> BlockServo.setPosition(0.82))
                         .lineTo(new Vector2d(73, 0))
                         .splineTo(new Vector2d(highGoal_x_first, highGoal_y_first), Math.toRadians(-45)) // new fluid movement
                         .waitSeconds(.1)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                         .back(8)
                         .UNSTABLE_addTemporalMarkerOffset(.0, () -> BlockServo.setPosition(0))
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .lineToSplineHeading(new Pose2d(74, -50, Math.toRadians(-45)))
                         .lineToSplineHeading(new Pose2d(89, -72, Math.toRadians(270)))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.63))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(300))
                         .back(10)
                         .strafeRight(15)
                         .lineToLinearHeading(new Pose2d(74, -22, Math.toRadians(0)))
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .build();

                 if (!isStopRequested())
                     drive.followTrajectorySequence(trajSeq);

             } else if (vision == 3) {

                 Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

                 drive.setPoseEstimate(startPose);

                 TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                         .addTemporalMarker(0.7, () -> SlideMotor.setTargetPosition(3900))
                         .addTemporalMarker(1.2, () -> BlockServo.setPosition(0.82))
                         .lineTo(new Vector2d(73, 0))
                         .splineTo(new Vector2d(highGoal_x_first, highGoal_y_first), Math.toRadians(-45)) // new fluid movement
                         .waitSeconds(.1)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                         .back(8)
                         .UNSTABLE_addTemporalMarkerOffset(.0, () -> BlockServo.setPosition(0))
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .lineToSplineHeading(new Pose2d(74, -50, Math.toRadians(-45)))
                         .lineToSplineHeading(new Pose2d(89, -72, Math.toRadians(270)))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.63))
                         .waitSeconds(.5)
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(300))
                         .back(10)
                         .strafeRight(15)
                         .lineToSplineHeading(new Pose2d(74, -44, Math.toRadians(0))) //park in 3
                         .UNSTABLE_addTemporalMarkerOffset(.1, () -> SlideMotor.setTargetPosition(0))
                         .build();

                 if (!isStopRequested())
                     drive.followTrajectorySequence(trajSeq);

             }








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