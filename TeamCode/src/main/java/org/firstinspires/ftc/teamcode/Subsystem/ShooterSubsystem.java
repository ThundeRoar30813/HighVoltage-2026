package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSubsystem {

    public DcMotorEx left;
    public DcMotorEx right;

    public void init(com.qualcomm.robotcore.hardware.HardwareMap hw) {

        left = hw.get(DcMotorEx.class, "school");
        right = hw.get(DcMotorEx.class, "shore");

        right.setDirection(DcMotorEx.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        left.setVelocityPIDFCoefficients(55.4, 0, 9.3, 246.6);
        right.setVelocityPIDFCoefficients(56.4, 0, 9.3, 246.6);
    }

    public void setVelocity(double vel) {
        left.setVelocity(vel);
        right.setVelocity(vel);
    }

    public void stop() {
        left.setVelocity(0);
        right.setVelocity(0);
    }

    public double getAvgVelocity() {
        return (left.getVelocity() + right.getVelocity()) / 2.0;
    }
}