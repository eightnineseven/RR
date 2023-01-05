//Base for autocode. Need to add rr
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class 1+x extends LinearOpMode
{
    static final double COUNTS_PER_MOTOR_REV = 384.5;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

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
                    if(tag.id == ID_TAG_OF_INTEREST)
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
















        //1 + x for stacking cones
        int x = 3;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(31.00, -65.00,  Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        // Next, notice the .strafeRight(10) and .forward(5) functions. We are telling the trajectory that
        // we want to strafe right 10 inches, then go forward 5 inches
        // more builds: https://learnroadrunner.com/trajectorybuilder-functions.html#straferight-distance-double

        //help on coordiantes:file:///C:/Users/Jack%20Scafidi/Downloads/FTC_FieldCoordinateSystemDefinition.pdf
        // and https://learnroadrunner.com/trajectories.html#coordinate-system

        //field layout: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-traditional.pdf

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Vector2d(31,-7), 45.00)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Vector2d(65, -13.5), 45)
                .build();

        Trajectory LeftPark = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Vector2d(12, -12), 180)
                .build();

        Trajectory RightPark = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Vector2d(12, -60), 18)
                .build();

        Trajectory CenterPark = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Vector2d(12, -36), 180)
                .build();




        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        arm(30);
        servo(OPEN);
        arm(0;)
        for(int i = 0; i < x; i++){
            drive.followTrajectory(traj1);
            servo(CLOSED);
            arm(30);
            drive.followTrajectory(traj2);
            servo(OPEN);
            arm(0);
        }

        park();

        PoseStorage.currentPose = drive.getPoseEstimate();















        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    //used for actual movement
    public static void servo(String servo){
        if(servo == "OPEN"){
            LeftServo.setPosition(0.85);
            RightServo.setPosition(0.85);
        } else {
            LeftServo.setPosition(1);
            RightServo.setPosition(1);
        }
    }
    public static void arm(int armInch){
        int newLiftTarget;
        if(opModeIsActive()){
            newLiftTarget = lift.getCurrentPosition() + (int)(armInch * COUNTS_PER_MOTOR_REV);
            lift.setTargetPosition(newLiftTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            lift.setPower(0.3);
            while(opModeIsActive() && lift.isBusy){
                if(lift.getCurrentPosition() == newLiftTarget){
                    lift.setPower(0);
        }
            }
        }
    }
    public static void park(){
        if(TagOfInterest == Left){
            drive.followTrajectory(LeftPark);
        } else if(TagOfInterest == Center){
            drive.followTrajectory(CenterPark);
        } else{
            drive.followTrajectory(RightPark);
        }
    }

}