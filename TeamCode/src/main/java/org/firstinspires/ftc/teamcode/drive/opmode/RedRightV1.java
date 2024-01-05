package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "RedRightV1")
public class RedRightV1 extends LinearOpMode {

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
        startPose = new Pose2d(-65,-15, Math.toRadians(0));
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
            while (true) {
                myTfodRecognitions = myTfodProcessor.getRecognitions();
                int noOfObjects = myTfodRecognitions.size();
                if (noOfObjects > 0) {
                    break;
                }
                sleep(1000);
                telemetry.addLine("Waiting to detect");
            }
            int position = getPosition(myTfodRecognitions);
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
        myTfodProcessorBuilder.setModelFileName("model_21877_RedCup.tflite");
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
                .lineTo(new Vector2d(-46, -15), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(-135))
                .lineTo(new Vector2d(-30, -52))
                //.splineToLinearHeading(new Pose2d(-30, -44),Math.toRadians(180))
                .addTemporalMarker(2, () -> {
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(340);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })

                .build();
        drive.followTrajectorySequence(goToBoard);

        sleep(300);
        wristServo.setPosition(0.38);
        sleep(200);
        clawRightServo.setPosition(0.55);
        sleep(100);

        //parking
        TrajectorySequence parking = drive.trajectorySequenceBuilder(goToBoard.end())
                .lineTo(new Vector2d(-30, -48), setSpeed(20), setAccelatation())
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    armMotor.setPower(1);
                    armMotor.setTargetPosition(500);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .lineTo(new Vector2d(-62, -48), setSpeed(40), setAccelatation())
                //.splineToLinearHeading(new Pose2d(-30, -62), -90)
                .build();
        drive.followTrajectorySequence(parking);
    }

    private void doTaskForPosition2(){
        TrajectorySequence testingByTurning = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,12), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(0))
                //drop hex
                //turn around
                //move to board
                //flip
                //place hex
                //park
                .build();
       // drive.followTrajectorySequence(testingByTurning);
    }

    private void doTaskForPosition3(){
        TrajectorySequence testingByTurning = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,12), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(90))
                //drop hex
                //turn around
                //move to board
                //flip
                //place hex
                //park
                .build();
        // drive.followTrajectorySequence(testingByTurning);
    }

    private TrajectoryVelocityConstraint setSpeed(int speed) {
        return SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    private TrajectoryAccelerationConstraint setAccelatation() {
        return SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }


}