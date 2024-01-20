package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.Servo;

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
}
