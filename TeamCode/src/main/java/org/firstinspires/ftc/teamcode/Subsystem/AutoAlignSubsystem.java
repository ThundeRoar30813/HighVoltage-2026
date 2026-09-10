package org.firstinspires.ftc.teamcode.Subsystem;

public class AutoAlignSubsystem {

    private double robotX = 50;
    private double robotY = 50;
    private double robotHeading = 0;

    private double targetX = 8;
    private double targetY = 136;

    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    public double calculateTurn() {

        double targetAngle = Math.atan2(targetY - robotY, targetX - robotX);
        double error = targetAngle - robotHeading;

        error = Math.atan2(Math.sin(error), Math.cos(error));

        double errorDeg = Math.toDegrees(Math.abs(error));

        if (errorDeg < 3) return 0;

        double speed = errorDeg / 180.0;
        speed = Math.max(0.09, Math.min(0.86, speed));

        return speed * Math.signum(error);
    }

    public void updatePose(double x, double y, double heading) {

        robotX = x;
        robotY = y;
        robotHeading = heading;
    }
}