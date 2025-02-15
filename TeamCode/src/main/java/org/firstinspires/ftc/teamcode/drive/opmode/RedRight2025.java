package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(name = "RedRight2025")
public class RedRight2025 extends LinearOpMode {


    SampleMecanumDrive drive;
    Pose2d startPose;
    private DcMotor viperSlideLeftMotor;
    private DcMotor viperSlideRightMotor;
    private DcMotor armMotor;
    private Servo clawLeftServo;
    private Servo clawRightServo;
    private Servo wristServo;

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


        clawLeftServo.setPosition(0.00);
        clawRightServo.setPosition(1.00);
        sleep(1000);
        wristServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setPosition(0.02);
        armMotor.setTargetPosition(0);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        // a custom TFLite object detection model.

        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        //Set initial position
        startPose = new Pose2d(-65, -10, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //if (opModeIsActive()) {
        // Put run blocks here.
            /*while (opModeIsActive()) {
                // Put loop blocks here.
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

        doTask();

        telemetry.addLine("Waiting before exiting");
        telemetry.update();
        sleep(10000);
        //}
    }


    private void doTask() {
        // Drop the hex
        //wristServo.setPosition(0.25);
        TrajectorySequence dropTheSample = drive.trajectorySequenceBuilder(startPose)


                .addTemporalMarker(0, () -> {
                    wristServo.setPosition(0.3);
                    moveViperslides(2600, 1);
                })
                .lineTo(new Vector2d(-35, -9.5), setSpeed(15), setAccelatation())
                .addTemporalMarker(2, () -> {
                    clawRightServo.setPosition(0.75);
                    clawLeftServo.setPosition(0.25);
                    moveViperslides(1800, 0.5);

                })
                .build();
        drive.followTrajectorySequence(dropTheSample);
        sleep(500);

        //Go to board
        TrajectorySequence goToSpecimen = drive.trajectorySequenceBuilder(dropTheSample.end())
                .lineTo(new Vector2d(-55, -10), setSpeed(20), setAccelatation())
                //.turn(Math.toRadians(-90))
                .addTemporalMarker(1, () -> {
                    wristServo.setPosition(0.05);
                    moveViperslides(0, 0.5);
                })
                //.lineTo(new Vector2d(-50, -23), setSpeed(20), setAccelatation())
                .lineToLinearHeading(new Pose2d(-62, -50, Math.toRadians(-90)), setSpeed(30), setAccelatation())
                .addTemporalMarker(2, () -> {
                    moveViperslides(1300, 1);
                    wristServo.setPosition(0.35);
                })
                .lineToLinearHeading(new Pose2d(-60, -56, Math.toRadians(-90)), setSpeed(10), setAccelatation())
                .addTemporalMarker(7, () -> {
                    clawRightServo.setPosition(1);
                    clawLeftServo.setPosition(0);
                    sleep(500);
                    moveViperslides(1400, 1);
                })


                //.lineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(25)), setSpeed(20), setAccelatation())
                .build();
        drive.followTrajectorySequence(goToSpecimen);


        TrajectorySequence dropTheSpecimen1 = drive.trajectorySequenceBuilder(goToSpecimen.end())

                .lineToLinearHeading(new Pose2d(-55, -9, Math.toRadians(0)), setSpeed(30), setAccelatation())

                .addTemporalMarker(0, () -> {
                    wristServo.setPosition(0.3);
                    moveViperslides(2750, 1);
                })
                .lineTo(new Vector2d(-37, -7.5), setSpeed(15), setAccelatation())
                .addTemporalMarker(7, () -> {
                    moveViperslides(1800, 0.5);
                })

                .lineTo(new Vector2d(-31.5, -9), setSpeed(15), setAccelatation())
                .addTemporalMarker(7, ()-> {
                    clawRightServo.setPosition(0.75);
                    clawLeftServo.setPosition(0.25);
                })

                .build();
       drive.followTrajectorySequence(dropTheSpecimen1);
        sleep(500);



    TrajectorySequence goToSpecimen2 = drive.trajectorySequenceBuilder(dropTheSpecimen1.end())
            .lineTo(new Vector2d(-55, -9), setSpeed(20), setAccelatation())
            //.turn(Math.toRadians(-90))
            .addTemporalMarker(1, () -> {
                wristServo.setPosition(0.05);
                moveViperslides(0, 0.5);
            })
            //.lineTo(new Vector2d(-50, -23), setSpeed(20), setAccelatation())
            .lineToLinearHeading(new Pose2d(-62, -50, Math.toRadians(-90)), setSpeed(20), setAccelatation())
            .addTemporalMarker(2, () -> {
                moveViperslides(1300, 1);
                wristServo.setPosition(0.35);
            })
            .lineToLinearHeading(new Pose2d(-63, -56, Math.toRadians(-90)), setSpeed(10), setAccelatation())
            .addTemporalMarker(7, () -> {
                clawRightServo.setPosition(1);
                clawLeftServo.setPosition(0);
                sleep(500);
                moveViperslides(1400, 1);
            })
                    .build();
            drive.followTrajectorySequence(goToSpecimen2);

        TrajectorySequence dropTheSpecimen2 = drive.trajectorySequenceBuilder(goToSpecimen2.end())

                .lineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(0)), setSpeed(30), setAccelatation())

                .addTemporalMarker(0, () -> {
                    wristServo.setPosition(0.3);
                    moveViperslides(2750, 1);
                })
                .lineTo(new Vector2d(-37, -7.5), setSpeed(15), setAccelatation())
                .addTemporalMarker(7, () -> {
                    moveViperslides(1800, 0.5);
                })

                .lineTo(new Vector2d(-31.5, -6), setSpeed(15), setAccelatation())
                .addTemporalMarker(7, ()-> {
                    clawRightServo.setPosition(0.75);
                    clawLeftServo.setPosition(0.25);
                })

                .build();
        drive.followTrajectorySequence(dropTheSpecimen2);

        TrajectorySequence pushTheSamples = drive.trajectorySequenceBuilder(dropTheSpecimen2.end())

                .lineTo(new Vector2d(-36, -6), setSpeed(15), setAccelatation())

                .addTemporalMarker(0, () -> {
                    wristServo.setPosition(0.05);
                    moveViperslides(0, 1);
                })

                .lineTo(new Vector2d(-36, -36), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-3, -36), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-3, -42), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-50, -42), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-3, -42), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-3, -55), setSpeed(15), setAccelatation())
                .lineTo(new Vector2d(-50, -55), setSpeed(15), setAccelatation())

                .build();
        drive.followTrajectorySequence(pushTheSamples);


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