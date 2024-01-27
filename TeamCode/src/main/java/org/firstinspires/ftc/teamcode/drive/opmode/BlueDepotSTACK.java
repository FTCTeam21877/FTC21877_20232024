package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import urmom;
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

@Autonomous(name = "BlueDepotSTACK")

public class BlueDepotSTACK extends LinearOpMode {
    //ARNAV KIRSHU PRANAV
    private DcMotor LeftLinearSlide;
    private DcMotor RightLinearSlide;
    private DcMotor LeftBack;
    private DcMotor LeftFront;

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
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Dicky = hardwareMap.get(Servo.class,"Dicky");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        BoxWrist.setPosition(0.14);
        DroneLauncber.setPosition(0.5);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Dicky.setPosition(0);
        LeftBox.setPosition(1);
        RightBox.setPosition(0.96);
        Pose2d startPose = new Pose2d(65,34, Math.toRadians(180));



        drive.setPoseEstimate(startPose);
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start O pMode");
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
        position=3;
        if(position == 1){
            left(drive,startPose);
        }else if(position == 2){
            middle(drive,startPose);
        }else{
            rightSide(drive,startPose);
        }

    }

    private void rightSide(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.6);
        TrajectorySequence drop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(32, 33, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(drop);
        hard();
        //Intake.setPower(.4);
        //sleep(800);
        //Intake.setPower(0);

        TrajectorySequence goToDropingPose = drive.trajectorySequenceBuilder(drop.end())
                .lineToLinearHeading(new Pose2d(14, 30, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(goToDropingPose);


        TrajectorySequence goToPickUpStack = drive.trajectorySequenceBuilder(goToDropingPose.end())
                .lineTo(new Vector2d(14,52.8),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToPickUpStack);
        Dicky.setPosition(0.54);

        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(goToPickUpStack.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(13.85,48.5, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);

        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(12,-36))
                .lineTo(new Vector2d(12,-48))
                //.lineTo(new Vector2d(31,-58))
                .build();
        drive.followTrajectorySequence(goToStartBridge);
        RightBox.setPosition(0.7);
        LeftBox.setPosition(1);
        changingLinearSlides(1250,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                        .lineTo(new Vector2d(29, -56))
                        .back(2)
                        .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);

        //changingLinearSlides(1250,0.8,true, true);
//        TrajectorySequence strafeLeft = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
//                .forward(4)
//                .strafeLeft(22)
//                .back(4)
//                .addTemporalMarker(0.5, () ->{
//                    resetStuff();
//                })
//                .build();
//        drive.followTrajectorySequence(strafeLeft);

        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();
        TrajectorySequence strafeeee = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeLeft(17)
                .build();
        drive.followTrajectorySequence(strafeeee);


        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();
    }
    private void middle(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.6);
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24.25, 46, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(goToDroppingPose);

        //Intake.setPower(.5);
        //sleep(800);
        //Intake.setPower(0);
        hard();
//mcdonads
        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .lineToLinearHeading(new Pose2d(17, 49, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(13, 46, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(lineUp);

        TrajectorySequence goToPickUpStack = drive.trajectorySequenceBuilder(lineUp.end())
                .lineTo(new Vector2d(16,57.0),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToPickUpStack);
        Dicky.setPosition(0.54);
//suhurth vupplala
        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(goToPickUpStack.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(14,53, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);

        //stupid rouck

        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(12,36))
                .lineTo(new Vector2d(12,-48))
                //.lineTo(new Vector2d(41,-57))
                .build();
        drive.followTrajectorySequence(goToStartBridge);
        //stupid bozo
        changingLinearSlides(1250,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                .lineTo(new Vector2d(34, -55))
                .back(2)
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);

//
//

        // bozo
        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();
        TrajectorySequence strafe = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeLeft(25)
                .build();
        drive.followTrajectorySequence(strafe);
//  ur mom
        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();







//        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
//                .lineTo(new Vector2d(12,36))
//                .lineTo(new Vector2d(12,-48))
//                .lineTo(new Vector2d(36,-55))
//                .build();
//        drive.followTrajectorySequence(goToStartBridge);
//
//        LeftBox.setPosition(1);
//        RightBox.setPosition(0.7);
//
//        changingLinearSlides(1000,0.8,true, true);
//
//        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToStartBridge.end())
//                .forward(3)
//                .build();
//        drive.followTrajectorySequence(forwarddd);
//        resetStuff();
//        TrajectorySequence strafeeee = drive.trajectorySequenceBuilder(forwarddd.end())
//                .strafeLeft(17)
//                .build();
//        drive.followTrajectorySequence(strafeeee);
//
//
//        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
//        telemetry.update();
    }
    private void left(SampleMecanumDrive drive, Pose2d startPose) {
        Dicky.setPosition(0.6);
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(34, 33, Math.toRadians(270)))
                .forward(4)
                .build();
        drive.followTrajectorySequence(goToDroppingPose);
        hard();
        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .lineToLinearHeading(new Pose2d(13,46,Math.toRadians(90)))
                .lineTo(new Vector2d(17.25,56.75),setSpeed(10),setAccelatation())
                .build();
        drive.followTrajectorySequence(lineUp);
        Dicky.setPosition(0.54);
        sleep(100);


        TrajectorySequence pickUpExtra = drive.trajectorySequenceBuilder(lineUp.end())
                .back(8, setSpeed(25),setAccelatation())
                .addTemporalMarker(0.2,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.3,() ->{
                    Dicky.setPosition(0.54);
                })
                .addTemporalMarker(0.45, () -> {
                    Intake.setPower(-0.7);
                    hard();
                })
                .addTemporalMarker(1.5, () -> {
                    Intake.setPower(-0.7);
                })
                .lineToLinearHeading(new Pose2d(14,53.5, Math.toRadians(90)))
                //.forward(6.5, setSpeed(13), setAccelatation())
                .build();
        drive.followTrajectorySequence(pickUpExtra);
        Dicky.setPosition(0);

        TrajectorySequence goToStartBridge = drive.trajectorySequenceBuilder(pickUpExtra.end())
                .lineTo(new Vector2d(12,36))
                .lineTo(new Vector2d(12,-48))
                //.lineTo(new Vector2d(41,-57))
                .build();
        drive.followTrajectorySequence(goToStartBridge);

        changingLinearSlides(1250,0.8,true, false);
        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(goToStartBridge.end())
                .lineTo(new Vector2d(39, -55))
                .back(2)
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);

//
//

       // bozo
        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(3)
                .build();
        drive.followTrajectorySequence(forwarddd);
        resetStuff();
        TrajectorySequence strafeeee = drive.trajectorySequenceBuilder(forwarddd.end())
                .strafeLeft(25)
                .build();
        drive.followTrajectorySequence(strafeeee);

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
            BoxWrist.setPosition(0.45);
            sleep(300);
        }
        else{
            BoxWrist.setPosition(0.14);
            sleep(300);
        }



    }
    private void resetStuff(){
        BoxWrist.setPosition(0.12);
        LeftBox.setPosition(1);
        RightBox.setPosition(0.7);


        LeftLinearSlide.setTargetPosition(0);
        LeftLinearSlide.setPower(-0.8);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLinearSlide.setTargetPosition(0);
        RightLinearSlide.setPower(-0.8);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);




    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_20240124_182536.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("BlueCup"));
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
        int position = 1;
        float previousWidth = 20000;
        float previousHeight = 20000;
        float smallestX = 0;
        float smallestY = 0;
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
            if (currentWidth < previousWidth){
                previousWidth = currentWidth;
                previousHeight = currentHeight;
                smallestX = x;
                smallestY = y;
            }
            float left = myTfodRecognition.getLeft();
            telemetry.addData("- Left", JavaUtil.formatNumber(left, 0));
            if (smallestX > 100 && smallestX < 450) {
                position = 2;
            } else if (smallestX >= 450) {
                position = 3;
            } else{
                position = 1;
            }//215 582
            telemetry.addData("- Position", JavaUtil.formatNumber(smallestX, 0) );
            telemetry.update();
        }



        return position;
    }

    private void soft(){
        Dicky.setPosition(0.45);
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
