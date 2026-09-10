package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous
public class AutonomusPrimeRed extends OpMode {
    private DcMotor shootMotor1,shootMotor2;
    private DcMotor  intake1Motor, intake2Motor;
    private Follower follow;
    private Timer pathTimer, opModeTimer;

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follow = Constants.createFollower(hardwareMap);

        buildPaths();
        follow.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop() {
        follow.update();
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("x",follow.getPose().getX());
        telemetry.addData("y",follow.getPose().getY());
        telemetry.addData("heading", follow.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());


        intake1Motor = hardwareMap.get(DcMotor.class, "mIntake1");
        intake2Motor = hardwareMap.get(DcMotor.class, "mIntake2");
        shootMotor2 = hardwareMap.get(DcMotor.class, "school");
        shootMotor1 = hardwareMap.get(DcMotor.class, "schore");

        shootMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        INTAKE_ON,
        SHOOT,
        MOVILITY

    }

    PathState pathState;
    private final Pose startPose = new Pose(123.126605504, 122.86238532110092, Math.toRadians(35));
    private final Pose shootPose = new Pose(120.22018348623853, 118.70091743119266, Math.toRadians(35));
    private final Pose shooteo11 = new Pose(120.22018348623853, 118.70091743119266, Math.toRadians(35));
    private final Pose shooteo12 = new Pose(120.22018348623853, 118.70091743119266, Math.toRadians(35));
    private final Pose movility = new Pose(129.4678899082569, 100.73394495412843, Math.toRadians(0));


    private PathChain driveStationPosShootPos;
    private PathChain segundoCiclo;
    void buildPaths() {
        driveStationPosShootPos = follow.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .addPath(new BezierLine(shootPose, shooteo11))
                .addPath(new BezierLine(shooteo11, shooteo12))
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follow.followPath(driveStationPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(follow.isBusy()) {
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.SHOOT);
                }
                break;
            case SHOOT:
                if(follow.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        shootMotor1.setPower(1);
                        shootMotor2.setPower(1);
                        setPathState(PathState.INTAKE_ON);
                    }
                    break;
                }
            case INTAKE_ON:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0) {
                    shootMotor1.setPower(1);
                    shootMotor2.setPower(1);
                    intake1Motor.setPower(1);
                    intake2Motor.setPower(1);
                }
            case MOVILITY:
                if(follow.isBusy()) {
                    break;
                }
            default:
                telemetry.addLine("No State Comand");
                break;

        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
