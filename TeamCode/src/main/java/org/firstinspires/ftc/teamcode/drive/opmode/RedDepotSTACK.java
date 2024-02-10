package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RedDepotSTACK")
public class RedDepotSTACK extends LinearOpMode {
    //ARNAV KIRSHU PRANAV
    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;
    private DcMotor LeftBack;
    private DcMotor LeftFront;
    private Servo DroneWrist;

    boolean USE_WEBCAM;
    private Servo BoxWrist;
    private Servo LeftBox;
    private Servo Dicky;
    private Servo RightBox;
    private Servo DroneLauncber;
    private DcMotor Intake;
    private DcMotor RightFront;
    private DcMotor RightBack;

    VisionPortal myVisionPortal;

    TfodProcessor myTfodProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftLinearSlide = hardwareMap.get(DcMotor.class, "LeftLinearSlide");
        RightLinearSlide = hardwareMap.get(DcMotor.class, "RightLinearSlide");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        BoxWrist = hardwareMap.get(Servo.class, "BoxWrist");
        LeftBox = hardwareMap.get(Servo.class, "LeftBox");
        RightBox = hardwareMap.get(Servo.class, "RightBox");
        DroneLauncber = hardwareMap.get(Servo.class, "DroneLauncber");
        DroneWrist = hardwareMap.get(Servo.class, "DroneWrist");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        Dicky = hardwareMap.get(Servo.class,"Dicky");
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        BoxWrist.setPosition(0.14);
        LeftBox.setPosition(1);
        RightBox.setPosition(0.96);
        DroneLauncber.setPosition(1);
        DroneWrist.setPosition(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Dicky.setPosition(0);
        Pose2d startPose = new Pose2d(-65,34, Math.toRadians(0));



        drive.setPoseEstimate(startPose);
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        List<Recognition> myTfodRecognitions = null;
        while (true) {
            myTfodRecognitions = myTfodProcessor.getRecognitions();
            int noOfObjects = myTfodRecognitions.size();
            if (noOfObjects >= 0) {
                break;
            }
            sleep(1000);
            telemetry.addLine("Waiting to detect");
        }
        int position = -1;
        //Test position
        int degree = 0;
        waitForStart();
        while (position < 0) {
            sleep(1000);
            myTfodRecognitions = myTfodProcessor.getRecognitions();
            telemetry.addData("what opjject", getPosition(myTfodRecognitions));
            position = getPosition(myTfodRecognitions);
            telemetry.update();
        }
        //Test position

        if(position == 1){
            leftSide(drive,startPose);
        }else if(position == 2){
            middle(drive,startPose);
        }else{
            right(drive,startPose);
        }


    }

    private void leftSide(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.51);

        TrajectorySequence goToDropingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29, 32.39, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(goToDropingPose);

        hard();

        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDropingPose.end())
                .lineToLinearHeading(new Pose2d(-14, 34, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(lineUp);

        TrajectorySequence goToPickUpStack = drive.trajectorySequenceBuilder(lineUp.end())
                .lineTo(new Vector2d(-12.5,54.5),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToPickUpStack);
        Dicky.setPosition(0.45);

        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(goToPickUpStack.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(-13,53, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);

        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(-12,36))
                .lineTo(new Vector2d(-12,-46))
                //.lineTo(new Vector2d(41,-57))
                .build();
        drive.followTrajectorySequence(goToStartBridge);

        changingLinearSlides(1100,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                .lineTo(new Vector2d(-30, -55))
                .back(2)
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        sleep(500);
        changingLinearSlides(1300,0.8,true, false);

        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();


        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeRight(20)
                .addTemporalMarker(0.5, () -> {
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(strafeRight);

        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();
    }

    private void middle(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.51);
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29, 45, Math.toRadians(-90)))
                .build();
        drive.followTrajectorySequence(goToDroppingPose);

        hard();

        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .lineToLinearHeading(new Pose2d(-18, 49, Math.toRadians(-90)))
                .forward(2)
                .lineToLinearHeading(new Pose2d(-16, 46, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(lineUp);

        TrajectorySequence goToPickUpStack = drive.trajectorySequenceBuilder(lineUp.end())
                .lineTo(new Vector2d(-13.5,56),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToPickUpStack);
        Dicky.setPosition(0.45);


        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(goToPickUpStack.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(-12,53, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);
        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(-12,36))
                .lineTo(new Vector2d(-12,-46))
                //.lineTo(new Vector2d(41,-57))
                .build();
        drive.followTrajectorySequence(goToStartBridge);

        changingLinearSlides(1100,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                .lineTo(new Vector2d(-35, -55))
                .back(2)
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        sleep(500);
        changingLinearSlides(1300,0.8,true, false);


        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();

        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeRight(20)
                .addTemporalMarker(0.5, () -> {
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(strafeRight);

        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();
    }
    private void right(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.51);
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, 27, Math.toRadians(-50)))
                .forward(1)
                .build();
        drive.followTrajectorySequence(goToDroppingPose);

        hard();

        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .back(3)
                .lineToLinearHeading(new Pose2d(-11,48,Math.toRadians(90)))
                .lineTo(new Vector2d(-12,54),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(lineUp);
        Dicky.setPosition(0.45);
        sleep(100);

        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(lineUp.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.45);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(-12.5,53, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);

        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(-12,36))
                .lineTo(new Vector2d(-12,-48))
                //.lineTo(new Vector2d(41,-57))
                .build();
        drive.followTrajectorySequence(goToStartBridge);

        changingLinearSlides(1100,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                .lineTo(new Vector2d(-45, -55))
                .back(2)
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        sleep(500);
        changingLinearSlides(1300,0.8,true, false);

        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();
        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeRight(25)
                .addTemporalMarker(0.5, () -> {
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(strafeRight);

        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();
    }



    private void changingLinearSlides(int ticks, double power, boolean openWrist, boolean openFlappers){
        //open posotion
        if (openFlappers) {
            LeftBox.setPosition(0.81);
            RightBox.setPosition(0.96);
            sleep(300);
        }
        else{
            LeftBox.setPosition(1);
            RightBox.setPosition(0.7);
            sleep(300);
        }

        LeftLinearSlide.setTargetPosition(ticks);
        LeftLinearSlide.setPower(power);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setTargetPosition(ticks);
        RightLinearSlide.setPower(power);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(openWrist) {
            BoxWrist.setPosition(0.41);
            sleep(300);
        }
        else{
            BoxWrist.setPosition(0.14);
            sleep(300);
        }



    }
    private void resetStuff(){
        BoxWrist.setPosition(0.14);
        LeftBox.setPosition(1);
        RightBox.setPosition(0.7);


        LeftLinearSlide.setTargetPosition(0);
        LeftLinearSlide.setPower(-0.8);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setTargetPosition(0);
        RightLinearSlide.setPower(-0.8);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(150);




    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_20240120_172231.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("RedCup"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private int getPosition(List<Recognition> myTfodRecognitions) {
        Recognition myTfodRecognition;
        float x = 1000;
        float y = 1000;
        int position = 3;
        float previousWidth = 20000;
        float previousHeight = 20000;
        float smallestX = 0;
        float smallestY = 0;
        float previousArea = 4000000;
        float left = 1300;
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
            float currentWidth = myTfodRecognition.getWidth();
            float currentHeight = myTfodRecognition.getHeight();
            float currentArea = currentHeight * currentWidth;
            if (currentArea < previousArea && currentArea < 25000) {
                previousWidth = currentWidth;
                previousHeight = currentHeight;
                previousArea = currentArea;
                smallestX = x;
                smallestY = y;
                left = myTfodRecognition.getLeft();
            }
            telemetry.addData("- Left", JavaUtil.formatNumber(left, 0));
        if (left < 200) {
            position = 1;
        } else if (left >= 200 && left < 1200) {
            position = 2;
        }
        else {
            position = 3;
        }
            telemetry.addData("- Position", JavaUtil.formatNumber(position, 0) );
            telemetry.update();
        }



        return position;
    }
    private void soft(){
        Dicky.setPosition(0.4);
        sleep(100);
    }
    private void hard(){
        Dicky.setPosition(0);
        sleep(100);
    }

    private TrajectoryVelocityConstraint setSpeed(int speed) {
        return SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    private TrajectoryAccelerationConstraint setAccelatation() {
        return SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }
}
