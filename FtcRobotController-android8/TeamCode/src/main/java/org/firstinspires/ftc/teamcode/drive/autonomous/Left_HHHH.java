package org.firstinspires.ftc.teamcode.drive.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous(group = "drive")
public class Left_HHHH extends LinearOpMode {
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
        ClawServo.setPosition(0.62);
        ClawServo.setDirection(Servo.Direction.REVERSE);
        ArmServo.setPosition(0);
        SlideMotor.setVelocity(10000);
        ClawServo.setPosition(0.60);
        BlockServo.setPosition(0);

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
            vision = 2;
        }else if(tagOfInterest.id == LEFT){
            vision = 1;
        }else if(tagOfInterest.id == MIDDLE){
            vision = 2;
        }else{
            vision = 3;
        }

        if (vision == 1) {
            //      --------- robot is 0, 0 ---------------
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

            drive.setPoseEstimate(startPose);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(0, () -> SlideMotor.setTargetPosition(3900))
                    .addTemporalMarker(.3, () -> BlockServo.setPosition(0.85))
                    .forward(39) // move straight forward
                    .lineToSplineHeading(new Pose2d(57, -2, Math.toRadians(-45))) //moving to high post
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                    .waitSeconds(.1)
                    // Go Back for 2nd Cone
                    .lineToSplineHeading(new Pose2d(52, 31, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(650)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)    //move forward 15 after pick up 2nd cone
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(57.5, -4, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #2 cones
                    .waitSeconds(.2)
                    // Go Back For 3rd Cone
                    .lineToSplineHeading(new Pose2d(54, 30, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(500)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #3 cones
                    .waitSeconds(.2)
                    // Go back for 4th cone
                    .lineToSplineHeading(new Pose2d(55, 29.5, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(350)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .waitSeconds(.5)
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.2)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #4 cones
                    .waitSeconds(.2)
                    // Prepare to park
                    .back(5)
                    .UNSTABLE_addTemporalMarkerOffset(.2, () -> ArmServo.setPosition(0.68)) // rotate claw arm to face
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(0))
                    .waitSeconds(.2)
                    .lineToSplineHeading(new Pose2d(46, -25.5, Math.toRadians(0))) // ready for teleop
                    .build();

            if (!isStopRequested())
                drive.followTrajectorySequence(trajSeq);

        } else if (vision == 2) {
            //      --------- robot is 0, 0 ---------------
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

            drive.setPoseEstimate(startPose);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(.5, () -> SlideMotor.setTargetPosition(3900))
                    .forward(39) // move straight forward
                    .lineToSplineHeading(new Pose2d(57, -2, Math.toRadians(-45))) //moving to high post
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                    .waitSeconds(.1)
                    // Go Back for 2nd Cone
                    .lineToSplineHeading(new Pose2d(52, 31, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(650)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)    //move forward 15 after pick up 2nd cone
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(57.5, -4, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #2 cones
                    .waitSeconds(.2)
                    // Go Back For 3rd Cone
                    .lineToSplineHeading(new Pose2d(54, 30, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(500)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #3 cones
                    .waitSeconds(.2)
                    // Go back for 4th cone
                    .lineToSplineHeading(new Pose2d(55, 29.5, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(350)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .waitSeconds(.5)
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.2)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #4 cones
                    .waitSeconds(.2)
                    // Prepare to park
                    .back(5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(0)) // lower slide
                    .lineToLinearHeading(new Pose2d(35, 3.5, Math.toRadians(180)))
                    .build();

            if (!isStopRequested())
                drive.followTrajectorySequence(trajSeq);

        } else if (vision == 3) {
            //      --------- robot is 0, 0 ---------------
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

            drive.setPoseEstimate(startPose);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .addTemporalMarker(.5, () -> SlideMotor.setTargetPosition(3900))
                    .forward(39) // move straight forward
                    .lineToSplineHeading(new Pose2d(57, -2, Math.toRadians(-45))) //moving to high post
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> ClawServo.setPosition(0.85)) // open claw lol
                    .waitSeconds(.1)
                    // Go Back for 2nd Cone
                    .lineToSplineHeading(new Pose2d(52, 31, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(650)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)    //move forward 15 after pick up 2nd cone
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(57.5, -4, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #2 cones
                    .waitSeconds(.2)
                    // Go Back For 3rd Cone
                    .lineToSplineHeading(new Pose2d(54, 30, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(500)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #3 cones
                    .waitSeconds(.2)
                    // Go back for 4th cone
                    .lineToSplineHeading(new Pose2d(55, 29.5, Math.toRadians(-90))) // back up to cone stack
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(350)) // slide height lower
                    .waitSeconds(.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.62)) // close claw lol
                    .waitSeconds(.4)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> SlideMotor.setTargetPosition(4000)) // slide height raise
                    //.waitSeconds(.5)
                    .forward(15)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> ArmServo.setPosition(0)) // rotate claw arm to face post
                    .waitSeconds(.5)
                    .lineToSplineHeading(new Pose2d(59.5, -4.5, Math.toRadians(-45))) // moving back to high post
                    .waitSeconds(.2)

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ClawServo.setPosition(0.85)) // open claw lol #4 cones
                    .waitSeconds(.2)

                    //park in 3
                    .lineToSplineHeading(new Pose2d(54, 27, Math.toRadians(-90))) //park in 3
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> ArmServo.setPosition(0.68)) // rotate claw arm
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> SlideMotor.setTargetPosition(0)) // set height for teleop
                    .waitSeconds(.3)
                    .build();

            if (!isStopRequested())
                drive.followTrajectorySequence(trajSeq);
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