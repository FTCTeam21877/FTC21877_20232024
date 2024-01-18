package org.firstinspires.ftc.teamcode.drive.opmode;

import android.hardware.Sensor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

@Autonomous(name = "BlueRightV1")
public class SensorTest extends LinearOpMode {

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

    private DistanceSensor distanceSensor;


    SampleMecanumDrive drive;

    Pose2d startPose;
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

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        clawLeftServo.setPosition(0.00);
        clawRightServo.setPosition(1.00);
        wristServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setPosition(0.03);
        launchServo.setPosition(0.50);
        // a custom TFLite object detection model.
        USE_WEBCAM = true;
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        //Set initial position
        startPose = new Pose2d(65, -13, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(60, -12), setSpeed(10), setAccelatation())
                //.turn(Math.toRadians(45))
                //.lineTo(new Vector2d(41, 28), setSpeed(20), setAccelatation())
                .build();
        drive.followTrajectorySequence(start);
        Direction direction = Direction.WEST;
        Pose2d currentPose = null;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                sleep(3000);
                currentPose = start.end();
                double x = currentPose.component1();
                double y = currentPose.component2();
                double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                telemetry.addData("Distance", distance);
                if (distance < 36) {
                    telemetry.addLine("Turning -90");
                    TrajectorySequence turnSequence = drive.trajectorySequenceBuilder(start.end())
                            .turn(Math.toRadians(-90))
                            .build();
                    drive.followTrajectorySequence(turnSequence);
                    start = turnSequence;
                    direction = turn(direction);
                }

                // Put loop blocks here.
                // Push telemetry to the Driver Station.
                TrajectorySequence dropTheHex = drive.trajectorySequenceBuilder(start.end())
                        .lineTo(getNewCoordinates(direction, start.end()), setSpeed(10), setAccelatation())
                        .build();
                drive.followTrajectorySequence(dropTheHex);

                telemetry.update();
            }
        }

    }

    private Direction turn(Direction direction) {
        if (Direction.WEST == direction) {
            return Direction.NORTH;
        } else if (Direction.NORTH == direction) {
            return Direction.EAST;
        } else if (Direction.EAST == direction) {
            return Direction.SOUTH;
        } else if (Direction.SOUTH == direction) {
            return Direction.WEST;
        }
        return null;
    }

    private Vector2d getNewCoordinates(Direction direction, Pose2d coordinates) {
        if (Direction.WEST == direction) {
            return new Vector2d(coordinates.component1()-24, coordinates.component2());
        } else if (Direction.NORTH == direction) {
            return new Vector2d(coordinates.component1(), coordinates.component2()+24);
        } else if (Direction.EAST == direction) {
            return new Vector2d(coordinates.component1()+24, coordinates.component2());
        } else if (Direction.SOUTH == direction) {
            return new Vector2d(coordinates.component1(), coordinates.component2()-24);
        }
        return null;
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


}