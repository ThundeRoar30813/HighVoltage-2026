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
public class AutonomusPrimeBlue extends OpMode {
    private DcMotor shootMotor1,shootMotor2;
    private DcMotor  intake1Motor, intake2Motor;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
    private final Pose startPose = new Pose(20.51452282157676, 123.68464730290458, Math.toRadians(144));
    private final Pose shootPose = new Pose(25.878899082568807, 117.12293577981653, Math.toRadians(144));
    private final Pose shooteo11 = new Pose(25.878899082568807, 110.46728971962615, Math.toRadians(144));
    private final Pose shooteo12 = new Pose(25.878899082568807, 110.46728971962615, Math.toRadians(144));
    private final Pose movility = new Pose(14.939449541284404, 99.94128440366973, Math.toRadians(180));


    private PathChain driveStationPosShootPos;
    private PathChain segundoCiclo;
    void buildPaths() {
        driveStationPosShootPos = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
            .addPath(new BezierLine(shootPose, shooteo11))
            .addPath(new BezierLine(shooteo11, shooteo12))
            .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStationPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(follower.isBusy()) {
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.SHOOT);
                }
                break;
            case SHOOT:
                if(follower.isBusy()) {
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
                if(follower.isBusy()) {
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
