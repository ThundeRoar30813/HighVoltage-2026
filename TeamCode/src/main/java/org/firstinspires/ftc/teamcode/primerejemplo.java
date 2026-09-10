package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Primer Ejemplo Auto", group = "Examples")
public class primerejemplo extends OpMode {

    private Follower follower;

    // Timers de PedroPathing
    private Timer pathTimer;
    private Timer opmodeTimer;

    // Estado actual del autónomo
    private int pathState = 0;

    // Poses del robot
    private final Pose startPose   = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose   = new Pose(60, 85, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));

    // Paths
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;
    private PathChain grabPickup2, scorePickup2;
    private PathChain grabPickup3, scorePickup3;

    /** Construye todos los paths del autónomo */
    public void buildPaths() {

        // Path para anotar el preload
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(
                startPose.getHeading(),
                scorePose.getHeading()
        );

        // Ir por el primer sample
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup1Pose.getHeading()
                )
                .build();

        // Anotar primer sample
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup1Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        // Ir por el segundo sample
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup2Pose.getHeading()
                )
                .build();

        // Anotar segundo sample
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup2Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();

        // Ir por el tercer sample
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(
                        scorePose.getHeading(),
                        pickup3Pose.getHeading()
                )
                .build();

        // Anotar tercer sample
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(
                        pickup3Pose.getHeading(),
                        scorePose.getHeading()
                )
                .build();
    }

    /** Máquina de estados del autónomo */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1); // Fin del autónomo
                }
                break;
        }
    }

    /** Cambia el estado del autónomo y reinicia el timer */
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    /** INIT */
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    /** LOOP */
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Estado", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {
        // No se necesita nada aquí
    }
}
