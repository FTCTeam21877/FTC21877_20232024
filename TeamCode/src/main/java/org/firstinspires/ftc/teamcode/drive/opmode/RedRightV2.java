package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RedRightV2")
public class RedRightV2 extends LinearOpMode {

    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;
    SampleMecanumDrive drive;
    Pose2d startPose;
    private DcMotor viperSlideLeftMotor;
    private DcMotor viperSlideRightMotor;
    private DcMotor armMotor;
    private Servo clawLeftServo;
    private Servo clawRightServo;
    private Servo wristServo;
    private Servo launchServo;

    private DistanceSensor distanceSensor;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        drive = new SampleMecanumDrive(hardwareMap);
        viperSlideLeftMotor = hardwareMap.get(DcMotor.class, "viperSlideleft");
        viperSlideLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperSlideRightMotor = hardwareMap.get(DcMotor.class, "viperslideright");
        viperSlideRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //viperSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawLeftServo = hardwareMap.get(Servo.class, "ClawLeft");
        clawRightServo = hardwareMap.get(Servo.class, "ClawRight");
        wristServo = hardwareMap.get(Servo.class, "WristServo");
        launchServo = hardwareMap.get(Servo.class, "plane");



        clawLeftServo.setPosition(0.00);
        clawRightServo.setPosition(1.00);
        sleep(1000);
        wristServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setPosition(0.03);
        launchServo.setPosition(0.50);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
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
        startPose = new Pose2d(-65, -13.75, Math.toRadians(0));
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
            //sleep(200);
            telemetry.addLine("Waiting to detect");
            totalWait += 100;
        }
        //int position = getPosition(myTfodRecognitions);
        int position = CircuitMakerUtils.getPositionByConfidence(myTfodRecognitions, telemetry);

        //int position =2;
        //Test position
        int degree = 0;
        if (position == 1) {
            //degree = -90;
            doTaskForPosition1V2();
        } else if (position == 2) {
            //degree = 0;
            doTaskForPosition2V2();
        } else {
            //degree = 90;
            doTaskForPosition3V2();
        }

        telemetry.addData("Postion", JavaUtil.formatNumber(position, 0));

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
        myTfodProcessorBuilder.setModelFileName("model_21877_RedCupWL.tflite");
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

            if (currentArea < previousArea && currentArea < 25000) {
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
        telemetry.addData("- Position", JavaUtil.formatNumber(position, 0));
        //telemetry.update();
        return position;
    }

    private void doTaskForPosition1() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                //.lineTo(new Vector2d(46, -13), setSpeed(30), setAccelatation())
                //.turn(Math.toRadians(45))
                .lineTo(new Vector2d(-45, -13.75))
                .lineToLinearHeading(new Pose2d(-41.5, -8.5,Math.toRadians(25)),setSpeed(40), setAccelatation())
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        sleep(300);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                //.lineTo(new Vector2d(46, -13), setSpeed(30), setAccelatation())
                //.turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-30, -48, Math.toRadians(-90)))
                .lineTo(new Vector2d(-30, -49.5), setSpeed(10), setAccelatation())

                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    moveArm(280, 1);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(200);
        wristServo.setPosition(0.38);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(200);

        //parking
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-30, -40))
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.27);
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    //wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.turn(Math.toRadians(180))
                .addTemporalMarker(1.0, () -> {
                    moveArm(0, 1);
                }).build();
        drive.followTrajectorySequence(goToStack);

        TrajectorySequence goToBoardLastLeg = null;
        if(distanceSensor.getDistance(DistanceUnit.INCH) > 48) {
            TrajectorySequence detectAndProceed = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-62, 36))
                    .addTemporalMarker(2, () -> {
                        wristServo.setPosition(0.28);
                        clawRightServo.setPosition(0.55);
                        clawLeftServo.setPosition(0.35);
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-39, 52))
                    .lineTo(new Vector2d(-39, 55.5), setSpeed(10), setAccelatation())
                    //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                    .build();
            drive.followTrajectorySequence(detectAndProceed);

            sleep(200);
            clawRightServo.setPosition(1);
            sleep(300);
            TrajectorySequence pickUp2ndHex = drive.trajectorySequenceBuilder(detectAndProceed.end())
                    .lineTo(new Vector2d(-39, 50))
                    .addTemporalMarker(0.5, () -> {
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-22.75, 50))
                    .lineTo(new Vector2d(-22.75, 55.5), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(pickUp2ndHex);
            sleep(200);
            clawLeftServo.setPosition(0.00);
            sleep(200);


            TrajectorySequence goToSecondBoard = drive.trajectorySequenceBuilder(pickUp2ndHex.end())
                    .lineTo(new Vector2d(-22.75, 54))
                    //.turn(Math.toRadians(90))
                    .addTemporalMarker(1, () -> {
                        wristServo.setPosition(0.03);
                        moveViperslides(0, 1);
                    })
                    .lineToLinearHeading(new Pose2d(-13, 49, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(-12, -42))
                    .build();
            drive.followTrajectorySequence(goToSecondBoard);

            //sleep(5000);
            wristServo.setPosition(0.27);
            sleep(200);
            moveArm(320, 1);
            sleep(200);

            goToBoardLastLeg = drive.trajectorySequenceBuilder(goToSecondBoard.end())
                    .lineTo(new Vector2d(-33, -49))
                    .lineTo(new Vector2d(-33, -51), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(goToBoardLastLeg);

            wristServo.setPosition(0.38);
            sleep(300);
            clawLeftServo.setPosition(0.35);
            clawRightServo.setPosition(0.55);
            sleep(200);

        } else {
            goToBoardLastLeg = goToStack;
        }
        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                //.lineTo(new Vector2d(-30, -45))
                .lineTo(new Vector2d(-33, -45))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    //moveArm(500, 1);
                })
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);


    }

    private void doTaskForPosition1V2() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-46, -15), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(45))
                .lineTo(new Vector2d(-40, -11), setSpeed(10), setAccelatation())
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        sleep(500);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                .lineTo(new Vector2d(-46, -15), setSpeed(30), setAccelatation())
                .turn(Math.toRadians(-135))
                .lineTo(new Vector2d(-30, -52))
                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(2, () -> {
                    moveArm(300, 1);

                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(300);
        wristServo.setPosition(0.4);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(100);

        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-30, -48), setSpeed(20), setAccelatation())
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                .lineTo(new Vector2d(-63, -48), setSpeed(40), setAccelatation())
                //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);

    }
    private void doTaskForPosition2() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-37, -15))
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        sleep(300);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                //.lineTo(new Vector2d(46, -13), setSpeed(30), setAccelatation())
                //.turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-34, -48, Math.toRadians(-90)))
                .lineTo(new Vector2d(-34, -49), setSpeed(10), setAccelatation())

                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    moveArm(280, 1);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(200);
        wristServo.setPosition(0.38);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(200);

        //Go to stack
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-34, -40))
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.03);
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    //wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.turn(Math.toRadians(180))
                .addTemporalMarker(1.0, () -> {
                    moveArm(0, 1);
                }).build();
        drive.followTrajectorySequence(goToStack);

        TrajectorySequence goToBoardLastLeg = null;
        if(distanceSensor.getDistance(DistanceUnit.INCH) > 48) {
            TrajectorySequence detectAndProceed = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-62, 36))
                    .addTemporalMarker(2, () -> {
                        wristServo.setPosition(0.28);
                        clawRightServo.setPosition(0.55);
                        clawLeftServo.setPosition(0.35);
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-39, 52))
                    .lineTo(new Vector2d(-39, 55.5), setSpeed(10), setAccelatation())
                    //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                    .build();
            drive.followTrajectorySequence(detectAndProceed);

            sleep(200);
            clawRightServo.setPosition(1);
            sleep(300);
            TrajectorySequence pickUp2ndHex = drive.trajectorySequenceBuilder(detectAndProceed.end())
                    .lineTo(new Vector2d(-39, 50))
                    .addTemporalMarker(0.5, () -> {
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-22.75, 50))
                    .lineTo(new Vector2d(-22.75, 55.5), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(pickUp2ndHex);
            sleep(200);
            clawLeftServo.setPosition(0.00);
            sleep(200);


            TrajectorySequence goToSecondBoard = drive.trajectorySequenceBuilder(pickUp2ndHex.end())
                    .lineTo(new Vector2d(-22.75, 54))
                    //.turn(Math.toRadians(90))
                    .addTemporalMarker(1, () -> {
                        wristServo.setPosition(0.03);
                        moveViperslides(0, 1);
                    })
                    .lineToLinearHeading(new Pose2d(-13, 49, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(-12, -42))
                    .build();
            drive.followTrajectorySequence(goToSecondBoard);

            //sleep(5000);
            wristServo.setPosition(0.27);
            sleep(200);
            moveArm(320, 1);
            sleep(200);

            goToBoardLastLeg = drive.trajectorySequenceBuilder(goToSecondBoard.end())
                    .lineTo(new Vector2d(-33, -49))
                    .lineTo(new Vector2d(-33, -51), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(goToBoardLastLeg);

            wristServo.setPosition(0.38);
            sleep(300);
            clawLeftServo.setPosition(0.35);
            clawRightServo.setPosition(0.55);
            sleep(200);

        } else {
            goToBoardLastLeg = goToStack;
        }
        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                //.lineTo(new Vector2d(-30, -45))
                .lineTo(new Vector2d(-33, -45))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    //moveArm(500, 1);
                })
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);


    }

    private void doTaskForPosition2V2() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-38, -20))
                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        //sleep(300);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                //.lineTo(new Vector2d(46, -13), setSpeed(30), setAccelatation())
                //.turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-35, -48, Math.toRadians(-90)))
                .lineTo(new Vector2d(-36, -49), setSpeed(10), setAccelatation())

                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    moveArm(280, 1);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(150);
        wristServo.setPosition(0.38);
        sleep(150);
        clawRightServo.setPosition(0.55);
        sleep(150);

        //Go to stack
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-34, -40))
                .lineToLinearHeading(new Pose2d(-60, -16, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.03);
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    //wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.turn(Math.toRadians(180))
                .addTemporalMarker(1.0, () -> {
                    moveArm(0, 1);
                }).build();
        drive.followTrajectorySequence(goToStack);

        //sleep(500);
        TrajectorySequence pickDirection = null;
        telemetry.addData("Distance:", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 48) {
            pickDirection = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-38, -12))
                    .lineTo(new Vector2d(-38, 36))
                    .addTemporalMarker(0, () -> {
                        wristServo.setPosition(0.03);
                    })
                    .addTemporalMarker(2.5, () -> {
                        wristServo.setPosition(0.28);
                        clawRightServo.setPosition(0.55);
                        clawLeftServo.setPosition(0.35);
                        moveViperslides(225, 1);
                    })
                    .build();
            drive.followTrajectorySequence(pickDirection);

            TrajectorySequence detectAndProceed = drive.trajectorySequenceBuilder(pickDirection.end())
                    .lineTo(new Vector2d(-39, 50))
                    .lineTo(new Vector2d(-39, 54), setSpeed(20), setAccelatation())
                    //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                    .build();
            drive.followTrajectorySequence(detectAndProceed);

            sleep(200);
            clawRightServo.setPosition(1);
            sleep(300);
            TrajectorySequence pickUp2ndHex = drive.trajectorySequenceBuilder(detectAndProceed.end())
                    .lineTo(new Vector2d(-39, 49))
                    .addTemporalMarker(0.5, () -> {
                        moveViperslides(235, 1);
                    })
                    .lineTo(new Vector2d(-22.75, 49))
                    .lineTo(new Vector2d(-22.75, 54), setSpeed(20), setAccelatation())
                    .build();
            drive.followTrajectorySequence(pickUp2ndHex);
            sleep(200);
            clawLeftServo.setPosition(0.00);
            sleep(200);


            TrajectorySequence goToSecondBoard = drive.trajectorySequenceBuilder(pickUp2ndHex.end())
                    .lineTo(new Vector2d(-22.75, 50))
                    //.turn(Math.toRadians(90))
                    .addTemporalMarker(0.75, () -> {
                        wristServo.setPosition(0.03);
                        moveViperslides(0, 1);
                    })
                    .lineToLinearHeading(new Pose2d(-13, 50, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(-11, -12))
                    .lineTo(new Vector2d(-11, -42))
                    .build();
            drive.followTrajectorySequence(goToSecondBoard);

            //sleep(5000);
            wristServo.setPosition(0.27);
            sleep(200);
            moveArm(320, 1);
            sleep(200);

            TrajectorySequence goToBoardLastLeg = drive.trajectorySequenceBuilder(goToSecondBoard.end())
                    .lineTo(new Vector2d(-43, -49))
                    .lineTo(new Vector2d(-43  , -53), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(goToBoardLastLeg);

            wristServo.setPosition(0.38);
            sleep(200);
            clawLeftServo.setPosition(0.35);
            clawRightServo.setPosition(0.55);
            sleep(200);

            //parking
            TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                    //.lineTo(new Vector2d(-30, -45))
                    .lineTo(new Vector2d(-43, -45))
                    .addTemporalMarker(1, () -> {
                        wristServo.setPosition(0.05);
                        //moveArm(500, 1);
                    })
                    .build();
            drive.followTrajectorySequence(parking);
            moveArm(0, 1);
        } else {
            //parking
            pickDirection = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-62, -45))
                    .build();
            drive.followTrajectorySequence(pickDirection);
        }
    }

    private void doTaskForPosition3() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-46, -27), setSpeed(30), setAccelatation())

                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        sleep(500);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                .lineTo(new Vector2d(-48, -27), setSpeed(20), setAccelatation())
                .lineToLinearHeading(new Pose2d(-41, -52, Math.toRadians(-90)))
                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    moveArm(300, 1);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(200);
        wristServo.setPosition(0.38);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(200);

        //parking
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-41, -40))
                .lineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.27);
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    //wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.turn(Math.toRadians(180))
                .addTemporalMarker(1.0, () -> {
                    moveArm(0, 1);
                }).build();
        drive.followTrajectorySequence(goToStack);

        TrajectorySequence goToBoardLastLeg = null;
        if(distanceSensor.getDistance(DistanceUnit.INCH) > 48) {
            TrajectorySequence detectAndProceed = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-62, 36))
                    .addTemporalMarker(2, () -> {
                        wristServo.setPosition(0.28);
                        clawRightServo.setPosition(0.55);
                        clawLeftServo.setPosition(0.35);
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-39, 52))
                    .lineTo(new Vector2d(-39, 55.5), setSpeed(10), setAccelatation())
                    //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                    .build();
            drive.followTrajectorySequence(detectAndProceed);

            sleep(200);
            clawRightServo.setPosition(1);
            sleep(300);
            TrajectorySequence pickUp2ndHex = drive.trajectorySequenceBuilder(detectAndProceed.end())
                    .lineTo(new Vector2d(-39, 50))
                    .addTemporalMarker(0.5, () -> {
                        moveViperslides(250, 1);
                    })
                    .lineTo(new Vector2d(-22.75, 50))
                    .lineTo(new Vector2d(-22.75, 55.5), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(pickUp2ndHex);
            sleep(200);
            clawLeftServo.setPosition(0.00);
            sleep(200);


            TrajectorySequence goToSecondBoard = drive.trajectorySequenceBuilder(pickUp2ndHex.end())
                    .lineTo(new Vector2d(-22.75, 54))
                    //.turn(Math.toRadians(90))
                    .addTemporalMarker(1, () -> {
                        wristServo.setPosition(0.03);
                        moveViperslides(0, 1);
                    })
                    .lineToLinearHeading(new Pose2d(-13, 49, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(-12, -42))
                    .build();
            drive.followTrajectorySequence(goToSecondBoard);

            //sleep(5000);
            wristServo.setPosition(0.27);
            sleep(200);
            moveArm(320, 1);
            sleep(200);

            goToBoardLastLeg = drive.trajectorySequenceBuilder(goToSecondBoard.end())
                    .lineTo(new Vector2d(-33, -49))
                    .lineTo(new Vector2d(-33, -51), setSpeed(10), setAccelatation())
                    .build();
            drive.followTrajectorySequence(goToBoardLastLeg);

            wristServo.setPosition(0.38);
            sleep(300);
            clawLeftServo.setPosition(0.35);
            clawRightServo.setPosition(0.55);
            sleep(200);

        } else {
            goToBoardLastLeg = goToStack;
        }
        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                //.lineTo(new Vector2d(-30, -45))
                .lineTo(new Vector2d(-33, -45))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    //moveArm(500, 1);
                })
                .build();
        drive.followTrajectorySequence(parking);
        moveArm(0, 1);


    }

    private void doTaskForPosition3V2() {
        // Drop the hex
        wristServo.setPosition(0.25);
        TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-46, -26))

                .build();
        drive.followTrajectorySequence(dropTheHex);
        clawLeftServo.setPosition(0.35);
        //sleep(200);
        wristServo.setPosition(0.20);
        sleep(200);

        //Go to board
        TrajectorySequence goToBoard = drive.trajectorySequenceBuilder(dropTheHex.end())
                .lineTo(new Vector2d(-48, -27))
                .lineToLinearHeading(new Pose2d(-41, -49, Math.toRadians(-90)))
                .lineTo(new Vector2d(-41, -50))

                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    moveArm(300, 1);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(100);
        wristServo.setPosition(0.38);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(200);

        //Go to stack
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-41, -45))
                .lineTo(new Vector2d(-50, -45))
                .lineToLinearHeading(new Pose2d(-60, -16, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.03);
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    //wristServo.setPosition(0.05);
                    moveArm(500, 1);
                })
                //.turn(Math.toRadians(180))
                .addTemporalMarker(1.0, () -> {
                    moveArm(0, 1);
                }).build();
        drive.followTrajectorySequence(goToStack);

        //sleep(500);
        TrajectorySequence pickDirection = null;
        telemetry.addData("Distance:", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        if(distanceSensor.getDistance(DistanceUnit.INCH) < 48) {
            pickDirection = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-38, -12))
                    .lineTo(new Vector2d(-38, 36))
                    .addTemporalMarker(0, () -> {
                        wristServo.setPosition(0.03);
                    })
                    .addTemporalMarker(2.5, () -> {
                        wristServo.setPosition(0.28);
                        clawRightServo.setPosition(0.55);
                        clawLeftServo.setPosition(0.35);
                        moveViperslides(225, 1);
                    })
                    .build();
            drive.followTrajectorySequence(pickDirection);

            TrajectorySequence detectAndProceed = drive.trajectorySequenceBuilder(pickDirection.end())
                    .lineTo(new Vector2d(-39, 50))
                    .lineTo(new Vector2d(-39, 54), setSpeed(20), setAccelatation())
                    //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                    .build();
            drive.followTrajectorySequence(detectAndProceed);

            sleep(200);
            clawRightServo.setPosition(1);
            sleep(200);
            TrajectorySequence pickUp2ndHex = drive.trajectorySequenceBuilder(detectAndProceed.end())
                    .lineTo(new Vector2d(-39, 50))
                    .addTemporalMarker(0.5, () -> {
                        moveViperslides(235, 1);
                    })
                    .lineTo(new Vector2d(-23, 50))
                    .lineTo(new Vector2d(-23, 53), setSpeed(20), setAccelatation())
                    .build();
            drive.followTrajectorySequence(pickUp2ndHex);
            sleep(200);
            clawLeftServo.setPosition(0.00);
            sleep(200);


            TrajectorySequence goToSecondBoard = drive.trajectorySequenceBuilder(pickUp2ndHex.end())
                    .lineTo(new Vector2d(-23, 50))
                    //.turn(Math.toRadians(90))
                    .addTemporalMarker(0.75, () -> {
                        wristServo.setPosition(0.03);
                        moveViperslides(0, 1);
                    })
                    .lineToLinearHeading(new Pose2d(-13, 50, Math.toRadians(0)))
                    .turn(Math.toRadians(-90))
                    //.lineTo(new Vector2d(-11, -12))
                    .lineTo(new Vector2d(-11, -42))
                    .build();
            drive.followTrajectorySequence(goToSecondBoard);

            //sleep(5000);
            wristServo.setPosition(0.27);
            sleep(200);
            moveArm(320, 1);
            sleep(100);

            TrajectorySequence goToBoardLastLeg = drive.trajectorySequenceBuilder(goToSecondBoard.end())
                    .lineTo(new Vector2d(-34, -49))
                    .lineTo(new Vector2d(-34, -52), setSpeed(30), setAccelatation())
                    .build();
            drive.followTrajectorySequence(goToBoardLastLeg);

            wristServo.setPosition(0.38);
            sleep(200);
            clawLeftServo.setPosition(0.35);
            clawRightServo.setPosition(0.55);
            sleep(200);

            //parking
            TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoardLastLeg.end())

                    //.lineTo(new Vector2d(-30, -45))
                    .lineTo(new Vector2d(-33, -45))
                    .addTemporalMarker(1, () -> {
                        wristServo.setPosition(0.05);
                        //moveArm(500, 1);
                    })
                    .build();
            drive.followTrajectorySequence(parking);
            moveArm(0, 1);
        } else {
            //parking
            pickDirection = drive.trajectorySequenceBuilder(goToStack.end())
                    .lineTo(new Vector2d(-62, -45))
                    .build();
            drive.followTrajectorySequence(pickDirection);
        }
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