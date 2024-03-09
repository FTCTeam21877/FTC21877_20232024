package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "RedDepotSTACKstates")
public class RedDepotSTACKSTATESS extends LinearOpMode {
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

    private Servo leftCheek;
    private Servo rightCheek;
    private DistanceSensor backDistanceSensor;
    private DistanceSensor frontDistanceSensor;

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
        leftCheek = hardwareMap.get(Servo.class, "LeftCheek");
        rightCheek = hardwareMap.get(Servo.class, "RightCheek");
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceBack");
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class,"DistanceFront");
        LeftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        BoxWrist.setPosition(0.14);
        cheeksSpread();
        LeftBox.setPosition(1);
        RightBox.setPosition(0.96);
        DroneLauncber.setPosition(1);
        DroneWrist.setPosition(0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
          //  sleep(1000);
            telemetry.addLine("Waiting to detect");
        }
        int position = -1;
        //Test position
        int degree = 0;
        waitForStart();
        while (position < 0) {
          //  sleep(1000);
            myTfodRecognitions = myTfodProcessor.getRecognitions();
            telemetry.addData("what opjject", getPosition(myTfodRecognitions));
            position = getPosition(myTfodRecognitions);
            telemetry.update();
        }
        //Test position

//

        if(position == 1){
            leftSide(drive,startPose);
        }else if(position == 2){
            middle(drive,startPose);
        }else{
            right(drive,startPose);
        }


    }

    private void leftSide(SampleMecanumDrive drive, Pose2d startPose) {
        cheeksMiddle();
        cheeksClosed();

        TrajectorySequence goToDropingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29, 30, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(goToDropingPose);

        cheeksSpread();

        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDropingPose.end())
                .lineTo(new Vector2d(-14, 28.5))
                .lineTo(new Vector2d(-17,47))
                .lineTo(new Vector2d(-17,51),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(lineUp);

        Intake.setPower(-1);
        cheeksClosed();
        LeftBox.setPosition(0.81);
        //RightBox.setPosition(0.96);
        sleep(300);
        cheeksSpread();

        TrajectorySequence gotoBoard = drive.trajectorySequenceBuilder(lineUp.end())
                .back(4)
                .splineToConstantHeading(new Vector2d(-35,-49),Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(gotoBoard);



        changingLinearSlides(1100,0.8,true, false);
        sleep(350);

        double distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;

        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(gotoBoard.end())
                //  .lineTo(new Vector2d(-39, -54))
                .lineToLinearHeading(new Pose2d(-34,-50 - distanceFromBoard,Math.toRadians(90)),setSpeed(5),setAccelatation())
                .build();
//        while (backDistanceSensor.getDistance(DistanceUnit.INCH) > 2.5) {
//            LeftBack.setPower(-0.1);
//            LeftFront.setPower(-0.1);
//            RightBack.setPower(-0.1);
//            RightFront.setPower(-0.1);
//            telemetry.addData("DistanceFront", frontDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("DistanceBack", backDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
//        LeftBack.setPower(0);
//        LeftFront.setPower(0);
//        RightBack.setPower(0);
//        RightFront.setPower(0);
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(200);
//        Pose2d firstBoardPos  = new Pose2d(-40,-56,Math.toRadians(90));
//        drive.setPoseEstimate(firstBoardPos);
        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(4)
                .addTemporalMarker(.5,() ->{
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(forwarddd);


        TrajectorySequence goBackToStack = drive.trajectorySequenceBuilder(forwarddd.end())
                .setTangent(Math.toRadians(20))
                .splineToConstantHeading(new Vector2d(-18,48),Math.toRadians(100))
                .build();
        drive.followTrajectorySequence(goBackToStack);
        double distancearnovGUPp = frontDistanceSensor.getDistance(DistanceUnit.INCH);
        TrajectorySequence pickingUp2 = drive.trajectorySequenceBuilder(goBackToStack.end())
                .lineToLinearHeading(new Pose2d(-21,54,Math.toRadians(90))  )
                .build();
        drive.followTrajectorySequence(pickingUp2);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        Intake.setPower(-1);
        cheeksClosed();

        sleep(300);
        cheeksSpread();
        sleep(300);
        cheeksClosed();
        sleep(300);
        cheeksSpread();
        sleep(200);

        //Second Cycle

        TrajectorySequence gotoBoard2Point0 = drive.trajectorySequenceBuilder(goBackToStack.end())
                .back(4)
                .setTangent(Math.toRadians(280))
                .splineToConstantHeading(new Vector2d(-38,-50),Math.toRadians(200))
                .build();
        drive.followTrajectorySequence(gotoBoard2Point0);
        changingLinearSlides(1200,0.8,true, false);


        distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;
        TrajectorySequence goToBoardAfterSlides2point0 = drive.trajectorySequenceBuilder(gotoBoard2Point0.end())

                .lineToLinearHeading(new Pose2d(-38,-50 - distanceFromBoard,Math.toRadians(90)),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides2point0);
//        while (backDistanceSensor.getDistance(DistanceUnit.INCH) > 2.5) {
//            LeftBack.setPower(-0.1);
//            LeftFront.setPower(-0.1);
//            RightBack.setPower(-0.1);
//            RightFront.setPower(-0.1);
//            telemetry.addData("DistanceFront", frontDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("DistanceBack", backDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
//        LeftBack.setPower(0);
//        LeftFront.setPower(0);
//        RightBack.setPower(0);
//        RightFront.setPower(0);
//
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(200);
        //drive.setPoseEstimate(new Pose2d(-41,-56,Math.toRadians(90)));
        TrajectorySequence forwarddd2dot0 = drive.trajectorySequenceBuilder(goToBoardAfterSlides2point0.end())
                .forward(4)
                .build();
        drive.followTrajectorySequence(forwarddd2dot0);
        resetStuff();
        sleep(300);



        //third cycle
//        TrajectorySequence goBackToStack3dot0 = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
////                .setTangent(Math.toRadians(30))
////                .splineToConstantHeading(new Vector2d(-11,20),Math.toRadians(130))
//                .setTangent(Math.toRadians(20))
//                .splineToConstantHeading(new Vector2d(-12,50),Math.toRadians(100))
//                .lineToLinearHeading(new Pose2d(-15,55,Math.toRadians(90)))
//                .forward(2.5,setSpeed(10),setAccelatation())
//                .build();
//        drive.followTrajectorySequence(goBackToStack3dot0);
//        Intake.setPower(-1);
//        LeftBox.setPosition(0.81);
//        RightBox.setPosition(0.96);

//        TrajectorySequence gotopark = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
//                .lineTo(new Vector2d(-20,-57))
//                .build();
//        drive.followTrajectorySequence(gotopark);

//
//
//
//
//
//
//
//
//
//        resetStuff();

//        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(forwarddd.end())
//                .strafeRight(20)
//                .addTemporalMarker(0.5, () -> {
//                    resetStuff();
//                })
//                .build();
//        drive.followTrajectorySequence(strafeRight);

        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();


    }

    private void middle(SampleMecanumDrive drive, Pose2d startPose) {
        cheeksMiddle();
        cheeksClosed();
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-28, 46, Math.toRadians(-90)))
                .forward(1.5, setSpeed(20), setAccelatation())
                .build();
        drive.followTrajectorySequence(goToDroppingPose);

        cheeksSpread();
        TrajectorySequence lineUpWithStackee = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-15.9,51.5,Math.toRadians(90)), setSpeed(50),setAccelatation())

                .lineTo(new Vector2d(-17.5,55.5),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(lineUpWithStackee);
        Intake.setPower(-1);
        cheeksClosed();
        LeftBox.setPosition(0.81);
        //RightBox.setPosition(0.96);
        sleep(300);
        cheeksSpread();
        TrajectorySequence gotoBoard = drive.trajectorySequenceBuilder(lineUpWithStackee.end())
                .back(4)
                .splineToConstantHeading(new Vector2d(-39,-48),Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(gotoBoard);



        changingLinearSlides(1100,0.8,true, false);
        sleep(350);

        double distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;

        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(gotoBoard.end())
              //  .lineTo(new Vector2d(-39, -54))
                .lineToLinearHeading(new Pose2d(-39,-50 - distanceFromBoard,Math.toRadians(90)),setSpeed(5),setAccelatation())
                .build();
//        while (backDistanceSensor.getDistance(DistanceUnit.INCH) > 2.5) {
//            LeftBack.setPower(-0.1);
//            LeftFront.setPower(-0.1);
//            RightBack.setPower(-0.1);
//            RightFront.setPower(-0.1);
//            telemetry.addData("DistanceFront", frontDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("DistanceBack", backDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
//        LeftBack.setPower(0);
//        LeftFront.setPower(0);
//        RightBack.setPower(0);
//        RightFront.setPower(0);
        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(200);
//        Pose2d firstBoardPos  = new Pose2d(-40,-56,Math.toRadians(90));
//        drive.setPoseEstimate(firstBoardPos);
        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(4)
                .addTemporalMarker(.5,() ->{
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(forwarddd);


        TrajectorySequence goBackToStack = drive.trajectorySequenceBuilder(forwarddd.end())
                .setTangent(Math.toRadians(20))
                .splineToConstantHeading(new Vector2d(-18,50),Math.toRadians(110))
                .build();
        drive.followTrajectorySequence(goBackToStack);
        double distancearnovGUPp = frontDistanceSensor.getDistance(DistanceUnit.INCH);
        TrajectorySequence pickingUp2 = drive.trajectorySequenceBuilder(goBackToStack.end())
                .lineToLinearHeading(new Pose2d(-20,57,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(pickingUp2);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        Intake.setPower(-1);
        cheeksClosed();

        sleep(300);
        cheeksSpread();
        sleep(300);
        cheeksClosed();
        sleep(300);
        cheeksSpread();
        sleep(200);

        //Second Cycle

        TrajectorySequence gotoBoard2Point0 = drive.trajectorySequenceBuilder(goBackToStack.end())
                .back(4)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-42,-50),Math.toRadians(210))
                .build();
        drive.followTrajectorySequence(gotoBoard2Point0);
        changingLinearSlides(1200,0.8,true, false);


        distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;
        TrajectorySequence goToBoardAfterSlides2point0 = drive.trajectorySequenceBuilder(gotoBoard2Point0.end())

                .lineToLinearHeading(new Pose2d(-42,-49 - distanceFromBoard,Math.toRadians(90)),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides2point0);
//        while (backDistanceSensor.getDistance(DistanceUnit.INCH) > 2.5) {
//            LeftBack.setPower(-0.1);
//            LeftFront.setPower(-0.1);
//            RightBack.setPower(-0.1);
//            RightFront.setPower(-0.1);
//            telemetry.addData("DistanceFront", frontDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("DistanceBack", backDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
//        LeftBack.setPower(0);
//        LeftFront.setPower(0);
//        RightBack.setPower(0);
//        RightFront.setPower(0);
//
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(200);
        //drive.setPoseEstimate(new Pose2d(-41,-56,Math.toRadians(90)));
        TrajectorySequence forwarddd2dot0 = drive.trajectorySequenceBuilder(goToBoardAfterSlides2point0.end())
                .forward(4)
                .build();
        drive.followTrajectorySequence(forwarddd2dot0);
        resetStuff();
        sleep(300);



        //third cycle
//        TrajectorySequence goBackToStack3dot0 = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
////                .setTangent(Math.toRadians(30))
////                .splineToConstantHeading(new Vector2d(-11,20),Math.toRadians(130))
//                .setTangent(Math.toRadians(20))
//                .splineToConstantHeading(new Vector2d(-12,50),Math.toRadians(100))
//                .lineToLinearHeading(new Pose2d(-15,55,Math.toRadians(90)))
//                .forward(2.5,setSpeed(10),setAccelatation())
//                .build();
//        drive.followTrajectorySequence(goBackToStack3dot0);
//        Intake.setPower(-1);
//        LeftBox.setPosition(0.81);
//        RightBox.setPosition(0.96);

//        TrajectorySequence gotopark = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
//                .lineTo(new Vector2d(-18.9,-57))
//                .build();
//        drive.followTrajectorySequence(gotopark);

//
//
//
//
//
//
//
//
//
//        resetStuff();

//        TrajectorySequence strafeRight = drive.trajectorySequenceBuilder(forwarddd.end())
//                .strafeRight(20)
//                .addTemporalMarker(0.5, () -> {
//                    resetStuff();
//                })
//                .build();
//        drive.followTrajectorySequence(strafeRight);

        telemetry.addData("Left  Slide Position", LeftLinearSlide.getCurrentPosition());
        telemetry.update();
    }
    private void right(SampleMecanumDrive drive, Pose2d startPose) {
        cheeksMiddle();
        cheeksClosed();
        TrajectorySequence goToDroppingPose = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 32.5, Math.toRadians(-90)))
                .forward(1.5, setSpeed(5), setAccelatation())
                .build();
        drive.followTrajectorySequence(goToDroppingPose);

        cheeksSpread();

        TrajectorySequence lineUp = drive.trajectorySequenceBuilder(goToDroppingPose.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-14,48,Math.toRadians(90)))
                .lineTo(new Vector2d(-16,54),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(lineUp);



        Intake.setPower(-1);
        cheeksClosed();
        LeftBox.setPosition(0.81);
        //RightBox.setPosition(0.96);
        sleep(300);
        cheeksSpread();
        TrajectorySequence gotoBoard = drive.trajectorySequenceBuilder(lineUp.end())
                .back(4)
                .splineToConstantHeading(new Vector2d(-43.5,-49),Math.toRadians(170))
                .build();
        drive.followTrajectorySequence(gotoBoard);



        changingLinearSlides(1100,0.8,true, false);
        sleep(350);

//        double distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;
//
//        if(distanceFromBoard > 15) {
//            distanceFromBoard = 10;
//        }

        TrajectorySequence goToBoardAfterSlides = drive.trajectorySequenceBuilder(gotoBoard.end())
                //  .lineTo(new Vector2d(-39, -54))
                .lineTo(new Vector2d(-43.5,-56),setSpeed(5),setAccelatation())
                .build();


        drive.followTrajectorySequence(goToBoardAfterSlides);
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(400);
//        Pose2d firstBoardPos  = new Pose2d(-40,-56,Math.toRadians(90));
//        drive.setPoseEstimate(firstBoardPos);
        TrajectorySequence forwarddd = drive.trajectorySequenceBuilder(goToBoardAfterSlides.end())
                .forward(4)
                .addTemporalMarker(.5,() ->{
                    resetStuff();
                })
                .build();
        drive.followTrajectorySequence(forwarddd);


        TrajectorySequence goBackToStack = drive.trajectorySequenceBuilder(forwarddd.end())
                .setTangent(Math.toRadians(20))
                .splineToConstantHeading(new Vector2d(-18,49),Math.toRadians(130))
                .build();
        drive.followTrajectorySequence(goBackToStack);
        double distancearnovGUPp = frontDistanceSensor.getDistance(DistanceUnit.INCH);
        TrajectorySequence pickingUp2 = drive.trajectorySequenceBuilder(goBackToStack.end())
                .lineToLinearHeading(new Pose2d(-19.5,55,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(pickingUp2);
        LeftBox.setPosition(0.81);
        RightBox.setPosition(0.96);
        Intake.setPower(-1);
        cheeksClosed();

        sleep(300);
        cheeksSpread();
        sleep(300);
        cheeksClosed();
        sleep(300);
        cheeksSpread();
        sleep(200);

        //Second Cycle

        TrajectorySequence gotoBoard2Point0 = drive.trajectorySequenceBuilder(pickingUp2.end())
                .back(4)
                .setTangent(Math.toRadians(280))
                .splineToConstantHeading(new Vector2d(-38,-50),Math.toRadians(210))
                .build();
        drive.followTrajectorySequence(gotoBoard2Point0);
        changingLinearSlides(1200,0.8,true, false);


//        distanceFromBoard = backDistanceSensor.getDistance(DistanceUnit.INCH) - 2.5;
//
//        if(distanceFromBoard > 15){
//            distanceFromBoard = 10;
//        }
        TrajectorySequence goToBoardAfterSlides2point0 = drive.trajectorySequenceBuilder(gotoBoard2Point0.end())

                .lineToLinearHeading(new Pose2d(-37,-56,Math.toRadians(90)),setSpeed(5),setAccelatation())
                .build();
        drive.followTrajectorySequence(goToBoardAfterSlides2point0);
//        while (backDistanceSensor.getDistance(DistanceUnit.INCH) > 2.5) {
//            LeftBack.setPower(-0.1);
//            LeftFront.setPower(-0.1);
//            RightBack.setPower(-0.1);
//            RightFront.setPower(-0.1);
//            telemetry.addData("DistanceFront", frontDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("DistanceBack", backDistanceSensor.getDistance(DistanceUnit.INCH));
//            telemetry.update();
//        }
//        LeftBack.setPower(0);
//        LeftFront.setPower(0);
//        RightBack.setPower(0);
//        RightFront.setPower(0);
//
        LeftBox.setPosition(0.81);
        sleep(200);
        RightBox.setPosition(0.96);
        sleep(200);
        //drive.setPoseEstimate(new Pose2d(-41,-56,Math.toRadians(90)));
        TrajectorySequence forwarddd2dot0 = drive.trajectorySequenceBuilder(goToBoardAfterSlides2point0.end())
                .forward(4)
                .build();
        drive.followTrajectorySequence(forwarddd2dot0);
        resetStuff();
        sleep(300);



        //third cycle
//        TrajectorySequence goBackToStack3dot0 = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
////                .setTangent(Math.toRadians(30))
////                .splineToConstantHeading(new Vector2d(-11,20),Math.toRadians(130))
//                .setTangent(Math.toRadians(20))
//                .splineToConstantHeading(new Vector2d(-12,50),Math.toRadians(100))
//                .lineToLinearHeading(new Pose2d(-15,55,Math.toRadians(90)))
//                .forward(2.5,setSpeed(10),setAccelatation())
//                .build();
//        drive.followTrajectorySequence(goBackToStack3dot0);
//        Intake.setPower(-1);
//        LeftBox.setPosition(0.81);
//        RightBox.setPosition(0.96);

//        TrajectorySequence gotopark = drive.trajectorySequenceBuilder(forwarddd2dot0.end())
//                .lineTo(new Vector2d(-20,-57))
//                .build();
//        drive.followTrajectorySequence(gotopark);

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
        sleep(400);




    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("RedCupV5.tflite");
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

        }
        if (left < 250) {
            position = 1;
        } else if (left >= 250 && left < 600) {
            position = 2;
        }
        else {
            position = 3;
        }
        telemetry.addData("- Left", JavaUtil.formatNumber(left, 0));

        telemetry.addData("- Position", JavaUtil.formatNumber(position, 0) );


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
    private void cheeksSpread(){
        leftCheek.setPosition(.49);
        rightCheek.setPosition(.46);
    }
    private void cheeksClosed(){
        leftCheek.setPosition(0);
        rightCheek.setPosition(1);
    }
    private void cheeksMiddle(){
        leftCheek.setPosition(.25);
        rightCheek.setPosition(.85);
        sleep(300);
    }

    private TrajectoryAccelerationConstraint setAccelatation() {
        return SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
    }
}
