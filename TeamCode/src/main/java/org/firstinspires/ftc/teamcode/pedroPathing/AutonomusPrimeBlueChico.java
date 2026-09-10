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
public class AutonomusPrimeBlueChico extends OpMode {
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
        MOVILITY

    }

    PathState pathState;
    private final Pose startPose = new Pose(59.97798165137614, 8.455045871559632, Math.toRadians(90));
    private final Pose shootPose = new Pose(14.796330275229359, 8.455045871559633, Math.toRadians(90));


    private PathChain driveStationPosShootPos;
    private PathChain segundoCiclo;
    void buildPaths() {
        driveStationPosShootPos = follower.pathBuilder()
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStationPosShootPos, true);
                break;
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