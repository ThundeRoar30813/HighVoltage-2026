package org.firstinspires.ftc.teamcode.opmodes.obsoloteos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.ftc.ActiveOpMode;
@Disabled
@TeleOp(name = "TeleOpSoloAutoAim", group = "Teleop")
public class TeleOpSoloAutoAim extends OpMode {
    private Follower follower;
    public double speedMultiplier;
    public double turnMultiplier = -0.7;
    public Pose target = new Pose(8,136);

    private int stableFrames = 0;
    public double alignMinSpeed = 0.1;
    public double alignMaxSpeed = 1.0;
    public double deadband = 3.0;
    private static final int REQUIRED_STABLE_FRAMES = 20;
    public boolean isAlignOn = false;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // posición inicial del robot
        follower.setStartingPose(new Pose(9, 9, 90));


    }
    public Command drive() {
        return new LambdaCommand()
                .setStart(() -> follower.startTeleopDrive(true))
                .setUpdate(() -> {
                    double turn;
                    if (ActiveOpMode.gamepad2().left_bumper) {
                        turnMultiplier = -0.35;
                        speedMultiplier = 0.25;
                    } else {
                        turnMultiplier = -0.7;
                        speedMultiplier = 1;
                    }
                    if (isAlignOn) {
                        turn = calculateAlignmentTurn();
                    } else {
                        stableFrames = 0;
                        turn = ActiveOpMode.gamepad2().right_stick_x * turnMultiplier;
                    }

                    follower.setTeleOpDrive(
                            -ActiveOpMode.gamepad2().left_stick_y * speedMultiplier,
                            -ActiveOpMode.gamepad2().left_stick_x * speedMultiplier,
                            turn,
                            false);
                })
                .setStop((Boolean interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                })
                .setIsDone(() -> false)
                .requires(this);
    }
    public Command autoAlign() {
        return new LambdaCommand()
                .setStart(() -> {
                    follower.startTeleopDrive(true);
                    resetFrames();
                })
                .setUpdate(() -> {
                    double turn = calculateAlignmentTurn();

                    follower.setTeleOpDrive(
                            0,
                            0,
                            turn,
                            false);

                })
                .setStop((Boolean interrupted) -> follower.breakFollowing())
                .setIsDone(this::isAtTargetHeading)
                .requires(this);
    }


    private double calculateAlignmentTurn() {
        double errorRad = getSignedError();
        double errorDeg = Math.abs(Math.toDegrees(errorRad));

        if (errorDeg < deadband) return 0;

        double speed = Math.max(alignMinSpeed, Math.min(alignMaxSpeed, errorDeg / 90));

        return speed * Math.signum(errorRad);
    }

    public double calculateHeading(Pose tempTarget) {
        Pose robotPose = follower.getPose();
        return Math.atan2(tempTarget.getY() - robotPose.getY(), tempTarget.getX() - robotPose.getX());
    }

    public double getSignedError() {
        double targetHeading = calculateHeading(target);
        double robotHeading = follower.getHeading();

        return Math.atan2(
                Math.sin(targetHeading - robotHeading),
                Math.cos(targetHeading - robotHeading)
        );
    }

    public double getError() {
        return Math.abs(getSignedError());
    }

    public boolean isAtTargetHeading() {
        if (Math.toDegrees(getError()) < deadband) {
            stableFrames++;
            return stableFrames >= REQUIRED_STABLE_FRAMES;
        } else {
            stableFrames = 0;
            return false;
        }
    }
    public double getDistanceToTarget() {
        return follower.getPose().distanceFrom(target);
    }
    public void resetFrames(){
        stableFrames = 0;
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();

        drive();

        if(gamepad1.dpadDownWasPressed()) {
            isAlignOn = true;
        }
        if (gamepad1.dpadUpWasPressed()) {
            isAlignOn = false;
        }
    }
}
