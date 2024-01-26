package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * Circuit makers utilities
 */
public class CircuitMakerUtils{

    /**
     * Set servo position
     * @param servo Servo Hardware
     * @param position Target Position
     * @param sleep Sleep value determines how slow or fast to make it move
     */
    public void setServoPosition(Servo servo, double position, long sleep) {
        while (true) {
            if (servo.getPosition() > position) {
                servo.setPosition(servo.getPosition() - 0.01);
            } else if (servo.getPosition() < position) {
                servo.setPosition(servo.getPosition() + 0.01);
            } else {
                return;
            }
            CircuitMakerUtils.sleep(sleep);
        }
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Get position by confidence
     */
    public static int getPositionByConfidence(List<Recognition> myTfodRecognitions, Telemetry telemetry) {
        Recognition myTfodRecognition;
        float x = 1000;
        float y = 1000;
        int position = 3;
        float previousWidth = 2000;
        float previousHeight = 2000;
        float previousArea = 4000000;
        float smallestX = 1000;
        float smallestY = 1000;
        float previousConfidence = 0;
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
            float currentConfidence = myTfodRecognition.getConfidence();
            telemetry.addData("- Area", JavaUtil.formatNumber(currentArea, 0));

            if (previousConfidence < currentConfidence){
                previousWidth = currentWidth;
                previousHeight = currentHeight;
                previousArea = currentArea;
                smallestX = x;
                smallestY = y;
                previousConfidence = currentConfidence;
            }

        }
        if (smallestX < 250) {
            position = 1;
        } else if (smallestX >= 250 && smallestX < 700) {
            position = 2;
        }
        telemetry.addData("- Position", JavaUtil.formatNumber(position, 0) );
        telemetry.update();
        return position;
    }
}
