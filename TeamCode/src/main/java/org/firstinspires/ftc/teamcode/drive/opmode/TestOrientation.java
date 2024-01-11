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

@Autonomous(name = "TestOrientation")
public class TestOrientation extends LinearOpMode {

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

        clawLeftServo.setPosition(0.39);
        clawRightServo.setPosition(0.55);
        wristServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setPosition(0.23);
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
        startPose = new Pose2d(0,0,Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence forward = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(0,30))
                .build();
        drive.followTrajectorySequence(forward);
        printPos();
        sleep(2000);

        TrajectorySequence backward = drive.trajectorySequenceBuilder(forward.end())
                .lineTo(new Vector2d(0,0))
                .build();
        drive.followTrajectorySequence(backward);
        printPos();
        sleep(2000);

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(backward.end())
                .lineTo(new Vector2d(10,0))
                .build();
        //drive.followTrajectorySequence(moveRight);
        printPos();
        sleep(2000);

        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(moveRight.end())
                .lineTo(new Vector2d(0,0))
                .build();
       // drive.followTrajectorySequence(moveLeft);
        printPos();
        sleep(2000);
    }
private void printPos(){
    Pose2d poseEstimate = drive.getPoseEstimate();
    telemetry.addData("x", poseEstimate.getX());
    telemetry.addData("y", poseEstimate.getY());
    telemetry.addData("heading", poseEstimate.getHeading());
    telemetry.update();
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
        myTfodProcessorBuilder.setModelFileName("model_21877_RedCupOld.tflite");
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
        float tempWidth = 20000;
        float tempHeight = 20000;
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

            if (myTfodRecognition.getWidth() < tempWidth){
                tempWidth = myTfodRecognition.getWidth();
                tempHeight = myTfodRecognition.getHeight();
                smallestX = x;
                smallestY = y;
            }

        }
        if (smallestX < 200) {
            position = 1;
        } else if (smallestX < 700) {
            position = 2;
        }
        telemetry.addData("- Position", JavaUtil.formatNumber(position, 0) );
        //telemetry.update();
        return position;
    }

    private void doTaskForPosition1(){
        TrajectorySequence testingByTurning = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,12), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(testingByTurning);
    }

    private void doTaskForPosition2(){
        TrajectorySequence testingByTurning = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,12), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(0))
                .build();
        drive.followTrajectorySequence(testingByTurning);
    }

    private void doTaskForPosition3(){
        TrajectorySequence testingByTurning = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,12), setSpeed(10), setAccelatation())
                .turn(Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(testingByTurning);
    }

    private TrajectoryVelocityConstraint setSpeed(int speed) {
        return SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    }

    private TrajectoryAccelerationConstraint setAccelatation() {
        return SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }


}