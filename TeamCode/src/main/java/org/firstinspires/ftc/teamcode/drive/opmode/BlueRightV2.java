package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@Autonomous(name = "BlueRightV2")
public class BlueRightV2 extends LinearOpMode {

    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;

    private DcMotor viperSlideLeftMotor;
    private DcMotor viperSlideRightMotor;

    private DcMotor armMotor;

    private Servo clawLeftServo;
    private Servo clawRightServo;

    private Servo wristServo;

    private Servo launchServo;

    SampleMecanumDrive drive;

    Pose2d startPose;
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        drive = new SampleMecanumDrive(hardwareMap);
        viperSlideLeftMotor = hardwareMap.get(DcMotor.class,"viperSlideleft");
        viperSlideLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlideRightMotor = hardwareMap.get(DcMotor.class,"viperslideright");
        viperSlideRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //viperSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotor.class,"ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLeftServo = hardwareMap.get(Servo.class, "ClawLeft");
        clawRightServo = hardwareMap.get(Servo.class,"ClawRight");
        wristServo = hardwareMap.get(Servo.class,"WristServo");
        launchServo = hardwareMap.get(Servo.class,"plane");

        clawLeftServo.setPosition(0.00);
        clawRightServo.setPosition(1.00);
        wristServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setPosition(0.03);
        launchServo.setPosition(0.50);
        // a custom TFLite object detection model.
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        //Set initial position
        startPose = new Pose2d(65,37.5, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //if (opModeIsActive()) {
            // Put run blocks here.
            /*while (opModeIsActive()) {
                // Put loop blocks here.
                telemetryTfod();
                // Push telemetry to the Driver Station.
                telemetry.update();
                if (gamepad1.dpad_down) {
                    // Temporarily stop the streaming session.
                    myVisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    // Resume the streaming session if previously stopped.
                    myVisionPortal.resumeStreaming();
                }
                // Share the CPU.
                sleep(20);
            }*/
            //Wait until object is detected
            List<Recognition> myTfodRecognitions = null;
            int maxWait = 1000;
            int totalWait = 0;
            while (true) {
                myTfodRecognitions = myTfodProcessor.getRecognitions();
                int noOfObjects = myTfodRecognitions.size();
                if (noOfObjects > 0 || totalWait > maxWait) {
                    break;
                }
                sleep(200);
                telemetry.addLine("Waiting to detect");
                totalWait +=200;
            }
            //int position = getPosition(myTfodRecognitions);
            int position = CircuitMakerUtils.getPositionByConfidence(myTfodRecognitions, telemetry);

            //Test position
            int degree = 0;
            if (position == 1) {
                //degree = -90;
                doTaskForPosition1();
            } else if (position == 2) {
                //degree = 0;
                doTaskForPosition2();
            } else {
                //degree = 90;
                doTaskForPosition3();
            }

        telemetry.addData("Postion", JavaUtil.formatNumber(position,0));

        telemetry.addLine("Waiting before exiting");
            telemetry.update();
            sleep(10000);
        //}
    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_21877_BlueCupWL.tflite");
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
        int position = 3;
        float previousWidth = 2000;
        float previousHeight = 2000;
        float previousArea = 4000000;
        float smallestX = 1000;
        float smallestY = 1000;
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
            float currentArea = currentWidth * currentHeight;
            telemetry.addData("- Area", JavaUtil.formatNumber(currentArea, 0));

            if (currentArea < previousArea && currentArea < 25000){
                previousWidth = currentWidth;
                previousHeight = currentHeight;
                previousArea = currentArea;
                smallestX = x;
                smallestY = y;
            }

        }
        if (smallestX < 250) {
            position = 1;
        } else if (smallestX >= 250 && smallestX < 700) {
            position = 2;
        }
        telemetry.addData("- Position", JavaUtil.formatNumber(position, 0) );
        //telemetry.update();
        return position;
    }

    private void doTaskForPosition1() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(46, 37.5))
                //.turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(40.25, 35, Math.toRadians(235)), setSpeed(20), setAccelatation())
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawRightServo.setPosition(0.5);
        moveViperslides(225,1);
        sleep(300);

        wristServo.setPosition(0.27);
        //wristServo.setPosition(0.15);
        //sleep(200);

        //Go to board
        TrajectorySequence priorToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                .lineTo(new Vector2d(45.5, 37.5))
                .lineToLinearHeading(new Pose2d(42.5, 54, Math.toRadians(110)), setSpeed(30), setAccelatation())
                .lineTo(new Vector2d(39.5,58.75), setSpeed(30), setAccelatation())
                .addTemporalMarker(0,()->{
                    //moveViperslides(125,1);
                    //wristServo.setPosition(0.30);
                })
                //.lineTo(new Vector2d(35.5,53))
                //.turn(Math.toRadians(-90))
                //.lineTo(new Vector2d(13, 52), setSpeed(30), setAccelatation())
                //.lineTo(new Vector2d(15, 33), setSpeed(30), setAccelatation())
                .build();
        drive.followTrajectorySequence(priorToBoard);
        sleep(200);
        clawRightServo.setPosition(1);
        sleep(300);


        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(priorToBoard.end())
                .lineTo(new Vector2d(42.5, 54))
                //.turn(Math.toRadians(90))
                .addTemporalMarker(0.5,()->{
                    moveViperslides(0,1);
                    wristServo.setPosition(0.03);
                })
                .lineToLinearHeading(new Pose2d(13, 49, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(13, -42))
                .build();
        drive.followTrajectorySequence(goToBoard);

        //sleep(5000);
        wristServo.setPosition(0.27);
        moveArm(320, 1);
        sleep(200);

        TrajectorySequence goToBoardLastLeg = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(42.5, -51))
                .build();
        drive.followTrajectorySequence(goToBoardLastLeg);

        wristServo.setPosition(0.38);
        sleep(300);
        clawLeftServo.setPosition(0.35);
        clawRightServo.setPosition(0.55);
        sleep(200);


        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                .lineTo(new Vector2d(42, -45))
                .lineTo(new Vector2d(20, -45))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.lineTo(new Vector2d(-60, -48), setSpeed(40), setAccelatation())
                //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);
    }

    private void doTaskForPosition2(){
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(37.5, 37.5))
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawRightServo.setPosition(0.5);
        //sleep(200);
        moveViperslides(225,1);
        sleep(300);

        wristServo.setPosition(0.27);
        //wristServo.setPosition(0.15);
        //sleep(200);

        //Go to board
        TrajectorySequence priorToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                .lineTo(new Vector2d(45.5, 37.5))
                .lineToLinearHeading(new Pose2d(42.5, 54, Math.toRadians(110)), setSpeed(30), setAccelatation())
                .lineTo(new Vector2d(39.25,59), setSpeed(30), setAccelatation())
                .addTemporalMarker(0,()->{
                    //moveViperslides(125,1);
                    //wristServo.setPosition(0.30);
                })
                //.lineTo(new Vector2d(35.5,53))
                //.turn(Math.toRadians(-90))
                //.lineTo(new Vector2d(13, 52), setSpeed(30), setAccelatation())
                //.lineTo(new Vector2d(15, 33), setSpeed(30), setAccelatation())
                .build();
        drive.followTrajectorySequence(priorToBoard);
        sleep(200);
        clawRightServo.setPosition(1);
        sleep(300);

        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(priorToBoard.end())
                .lineTo(new Vector2d(42.5, 54))
                //.turn(Math.toRadians(90))
                .addTemporalMarker(0.5,()->{
                    moveViperslides(0,1);
                })
                .lineToLinearHeading(new Pose2d(13, 49, Math.toRadians(180)), setSpeed(30), setAccelatation())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(13, -42), setSpeed(30), setAccelatation())
                .build();
        drive.followTrajectorySequence(goToBoard);

        //sleep(5000);
        moveArm(320, 1);
        sleep(100);

        TrajectorySequence goToBoardLastLeg = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(35, -51), setSpeed(30), setAccelatation())
                .build();
        drive.followTrajectorySequence(goToBoardLastLeg);

        wristServo.setPosition(0.38);
        sleep(300);
        clawLeftServo.setPosition(0.35);
        clawRightServo.setPosition(0.55);
        sleep(200);


        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                .lineTo(new Vector2d(35, -45), setSpeed(30), setAccelatation())
                .lineTo(new Vector2d(20, -45))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.lineTo(new Vector2d(-60, -48), setSpeed(40), setAccelatation())
                //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);


    }


    private void doTaskForPosition3(){
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                //.lineTo(new Vector2d(46, 41.5))
                //.turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(38, 39.5, Math.toRadians(135)), setSpeed(20), setAccelatation())
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawRightServo.setPosition(0.5);
        moveViperslides(225,1);
        sleep(300);

        wristServo.setPosition(0.27);
        //wristServo.setPosition(0.15);
        //sleep(200);

        //Go to board
        TrajectorySequence priorToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                //.lineTo(new Vector2d(40, 37.5))
                .lineToLinearHeading(new Pose2d(40, 37.5, Math.toRadians(180)), setSpeed(30), setAccelatation())
                .lineTo(new Vector2d(20,37.5))
                .lineToLinearHeading(new Pose2d(18.5, 54, Math.toRadians(110)), setSpeed(30), setAccelatation())
                .lineTo(new Vector2d(16.25,58.75), setSpeed(30), setAccelatation())
                .addTemporalMarker(0,()->{
                    //moveViperslides(125,1);
                    //wristServo.setPosition(0.30);
                })
                //.lineTo(new Vector2d(35.5,53))
                //.turn(Math.toRadians(-90))
                //.lineTo(new Vector2d(13, 52), setSpeed(30), setAccelatation())
                //.lineTo(new Vector2d(15, 33), setSpeed(30), setAccelatation())
                .build();
        drive.followTrajectorySequence(priorToBoard);
        sleep(200);
        clawRightServo.setPosition(1);
        sleep(300);


        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(priorToBoard.end())
                .lineTo(new Vector2d(18.5, 54))
                //.turn(Math.toRadians(90))
                .addTemporalMarker(0.5,()->{
                    moveViperslides(0,1);
                    wristServo.setPosition(0.03);
                })
                .lineToLinearHeading(new Pose2d(13, 49, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(13, -42))
                .build();
        drive.followTrajectorySequence(goToBoard);

        //sleep(5000);
        wristServo.setPosition(0.27);
        moveArm(280, 1);
        sleep(200);

        TrajectorySequence goToBoardLastLeg = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(32, -42))
                .lineTo(new Vector2d(32, -48))
                .build();
        drive.followTrajectorySequence(goToBoardLastLeg);

        wristServo.setPosition(0.38);
        sleep(300);
        clawLeftServo.setPosition(0.35);
        clawRightServo.setPosition(0.55);
        sleep(200);


        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                .lineTo(new Vector2d(32, -44))
                .lineTo(new Vector2d(20, -44))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.lineTo(new Vector2d(-60, -48), setSpeed(40), setAccelatation())
                //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);


    }

    private TrajectoryVelocityConstraint setSpeed(int speed) {
        return SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    private TrajectoryAccelerationConstraint setAccelatation() {
        return SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }

    private void moveArm(int targetPosition, double power) {
        armMotor.setPower(power);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void moveViperslides(int targetPosition, double power) {
        viperSlideLeftMotor.setPower(power);
        viperSlideLeftMotor.setTargetPosition(targetPosition);
        viperSlideLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideRightMotor.setPower(power);
        viperSlideRightMotor.setTargetPosition(targetPosition);
        viperSlideRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}