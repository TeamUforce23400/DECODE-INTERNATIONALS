package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.commands.allBallsDetected;
import org.firstinspires.ftc.teamcode.commands.allBallsGone;
import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
import org.firstinspires.ftc.teamcode.commands.isAimed;
import org.firstinspires.ftc.teamcode.commands.safeAllBallsDetected;
import org.firstinspires.ftc.teamcode.commands.setShooterFromPose;
import org.firstinspires.ftc.teamcode.commands.setTurretPos;
import org.firstinspires.ftc.teamcode.commands.shooterAtSpeed;
import org.firstinspires.ftc.teamcode.commands.slowTransfer;
import org.firstinspires.ftc.teamcode.commands.stationary;
import org.firstinspires.ftc.teamcode.commands.transfer;
import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@Autonomous(name = "Red Far Side 27 Path Auto")
public class RedFarSide extends CommandOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private static final double H0 = Math.toRadians(0);
    TimerEx matchTimer = new TimerEx(30);
    List<LynxModule> allHubs;
    private static final double H30 = Math.toRadians(30);
    private static final double H60 = Math.toRadians(60);
    private static final double H80 = Math.toRadians(80);
    private static final double H_NEG20 = Math.toRadians(-20);
    private AprilTagTracking vision;

    private final Pose startPose = new Pose(86.1, 6.74, Math.toRadians(0));
    private final Pose pose1 = new Pose(86.1, 16, Math.toRadians(0));
    private final Pose humanPlayerRight = new Pose(132, 7.8, Math.toRadians(50));
    private final Pose humanPlayerRightForward = new Pose(133, 50, Math.toRadians(40));
//    private final Pose nearMark = new Pose(120.128, 83.66);
//    private final Pose nearShoot = new Pose(94.608, 83.138);
//    private final Pose nearShootCP = new Pose(97.35, 66.32);
//    private final Pose collectMiddleMarkCP = new Pose(96.025, 62.109);
//    private final Pose collectMiddleMark = new Pose(126, 62.145);

    private final Pose humanPlayerRightFirst = new Pose(136.3, 4.8, Math.toRadians(0));

    private final Pose humanPlayerRightFirstDrifted = new Pose(138.3, 1.8, Math.toRadians(0));
    private final Pose collectLastMarkCP = new Pose(109.430, 37.845, Math.toRadians(-60));
    private final Pose collectLastMark = new Pose(128.360, 35.231);

    private final Pose collectOverflow = new Pose(130.5, 30.1, Math.toRadians(40));

    private final Pose poseAfterHumanPlayer = new Pose(85.8, 10, Math.toRadians(40));
//    private final Pose collectRamp = new Pose(135.7, 59.2, Math.toRadians(26.8)); // acc 59.4 y
//    private final Pose hp1 = new Pose(133.136, 27.088, Math.toRadians(-45));
//    private final Pose hp2 = new Pose(136.522, 11.0639, Math.toRadians(-90));

    private int loopCounter;
    private ElapsedTime elapsedtime;

    private PathChain path1,path2, path3, collectOverflowRamp, overflowShoot, path4, path5, path6, path7, path8, path9, path10, path10Mid, path10End, path11, path12, path13, path12Mid, path12End, path8Drifted, humanPlayerToShoot, humanPlayerCollectPathFinish, leave, path4Drifted, path5Drifted;

    private void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        pose1
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .15,
                                        HeadingInterpolator.linear(H0, pose1.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .15,
                                        1.0,
                                        HeadingInterpolator.constant(pose1.getHeading())
                                )
                        )
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pose1,
                        new Pose(humanPlayerRight.getX() - 5, humanPlayerRight.getY(), humanPlayerRight.getHeading())
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        1.0,
                                        HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(32))
                                )
                        )
                )
                .addPath(new BezierLine(
                        new Pose(humanPlayerRight.getX() - 5, humanPlayerRight.getY(), humanPlayerRight.getHeading()),
                        humanPlayerRightForward
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(75))
                                )
                        )
                )
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        humanPlayerRightForward,
                        poseAfterHumanPlayer
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(H0)
                                )
                        )
                )
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pose1,
                        humanPlayerRightFirst
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )
                .build();

        path4Drifted =  follower.pathBuilder()
                .addPath(new BezierLine(
                        pose1,
                        humanPlayerRightFirstDrifted
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        1.0,
                                        HeadingInterpolator.constant(Math.toRadians(0))
                                )
                        )
                )
                .build();

        path5Drifted = follower.pathBuilder()
                .addPath(new BezierLine(
                        humanPlayerRightFirstDrifted,
                        poseAfterHumanPlayer
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(H0)
                                )
                        )
                )
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        humanPlayerRightFirst,
                        poseAfterHumanPlayer
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(H0)
                                )
                        )
                )
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pose1, collectLastMarkCP
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .65,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .65,
                                        1.0,
                                        HeadingInterpolator.constant(H0)
                                )
                        ))
                .addPath(new BezierLine(
                        collectLastMarkCP, collectLastMark
                ))
                .setConstantHeadingInterpolation(H0)
                .addPath(new BezierLine(
                        collectLastMark, pose1
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.6,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.6,
                                        1,
                                        HeadingInterpolator.constant(H0)
                                )
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .8,
//                                        1.0,
//                                        HeadingInterpolator.constant(H0)
//                                )
                        ))
                .build();

        collectOverflowRamp = follower.pathBuilder()
                .addPath(new BezierLine(
                        poseAfterHumanPlayer,
                        collectOverflow
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .15,
                                        HeadingInterpolator.linear(H0, collectOverflow.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .15,
                                        1.0,
                                        HeadingInterpolator.constant(collectOverflow.getHeading())
                                )
                        )
                )


                .build();

        overflowShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        collectOverflow,
                        poseAfterHumanPlayer
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .15,
                                        HeadingInterpolator.linear(H0, poseAfterHumanPlayer.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .15,
                                        1.0,
                                        HeadingInterpolator.constant(poseAfterHumanPlayer.getHeading())
                                )
                        )
                )


                .build();


//        path3 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        collectMiddleMark,
//                        collectMiddleMarkCP,
//                        pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .7,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .7,
//                                        1.0,
//                                        HeadingInterpolator.constant(H0)
//                                )
//                        )
//                )
//                .build();
//
//        path4 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1,
//                        new Pose(collectRamp.getX(), collectRamp.getY() - 0.8, collectRamp.getHeading())
//
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .6,
//                                        HeadingInterpolator.tangent
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .6,
//                                        1.0,
//                                        HeadingInterpolator.constant(collectRamp.getHeading()))
//                        )
//                )
//                .build();
//
//        path5 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        collectRamp,
//                        nearShootCP,
//                        nearShoot
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .7,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .75,
//                                        1.0,
//                                        HeadingInterpolator.constant(H0))
//                        )
//                )
//                .build();
//
//        path6 = follower.pathBuilder()
//                .addPath(new BezierLine(nearShoot, nearMark))
//                .addPath(new BezierLine(
//                        nearMark,
//                        pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .7,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .7,
//                                        1.0,
//                                        HeadingInterpolator.constant(Math.toRadians(-10))
//                                )
//                        ))
//                .build();
//
////        path7 = follower.pathBuilder()
////                .addPath(new BezierLine(
////                        nearMark,
////                        pose1
////                ))
////                .setHeadingInterpolation(
////                        HeadingInterpolator.piecewise(
////                                new HeadingInterpolator.PiecewiseNode(
////                                        0,
////                                        .7,
////                                        HeadingInterpolator.tangent.reverse()
////                                ),
////                                new HeadingInterpolator.PiecewiseNode(
////                                        .7,
////                                        1.0,
////                                        HeadingInterpolator.constant(Math.toRadians(-10))
////                        )
////                ))
////                .build();
//        path8 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1,
//                        collectRamp
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .6,
//                                        HeadingInterpolator.tangent
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .6,
//                                        1.0,
//                                        HeadingInterpolator.constant(collectRamp.getHeading()))
//                        )
//                )
//                .build();
//
//        path8Drifted = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1,
//                        new Pose(collectRamp.getX(), collectRamp.getY() - 2.05, collectRamp.getHeading())
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .6,
//                                        HeadingInterpolator.tangent
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .6,
//                                        1.0,
//                                        HeadingInterpolator.constant(collectRamp.getHeading()))
//                        )
//                )
//                .build();
//
//        path9 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        collectRamp, pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .8,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .8,
//                                        1.0,
//                                        HeadingInterpolator.constant(Math.toRadians(-25))
//                                )
//                        ))
//                .build();
//
//        path10 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1, collectLastMarkCP
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .65,
//                                        HeadingInterpolator.tangent
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .65,
//                                        1.0,
//                                        HeadingInterpolator.constant(H0)
//                                )
//                        ))
//                .addPath(new BezierLine(
//                        collectLastMarkCP, collectLastMark
//                ))
//                .setConstantHeadingInterpolation(H0)
//                .addPath(new BezierLine(
//                        collectLastMark, pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        0.6,
//                                        HeadingInterpolator.tangent.reverse()
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0.6,
//                                        1,
//                                        HeadingInterpolator.constant(H0)
//                                )
////                                new HeadingInterpolator.PiecewiseNode(
////                                        .8,
////                                        1.0,
////                                        HeadingInterpolator.constant(H0)
////                                )
//                        ))
//                .build();
//
//        path10Mid = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1, collectLastMarkCP
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .8,
//                                        HeadingInterpolator.tangent
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .8,
//                                        1.0,
//                                        HeadingInterpolator.constant(H0)
//                                )
//                        ))
//                .build();
//        path10End = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        collectLastMarkCP, collectLastMark
//                ))
//                .setConstantHeadingInterpolation(H0)
//                .build();
//
//        path11 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        collectLastMark, pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        1.0,
//                                        HeadingInterpolator.tangent.reverse()
//                                )
////                                new HeadingInterpolator.PiecewiseNode(
////                                        .8,
////                                        1.0,
////                                        HeadingInterpolator.constant(H0)
////                                )
//                        ))
//                .build();
//
//        path12 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1, hp1
//                )).setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .8,
//                                        HeadingInterpolator.constant(hp1.getHeading())
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .8,
//                                        1.0,
//                                        HeadingInterpolator.linear(hp1.getHeading(), hp2.getHeading())
//                                )
//                        ))
//                .addPath(new BezierLine(
//                        hp1, hp2
//                ))
//                .setConstantHeadingInterpolation(hp2.getHeading())
//                .build();
//
//        path12Mid = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1, hp1
//                )).setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        .8,
//                                        HeadingInterpolator.constant(hp1.getHeading())
//                                ),
//                                new HeadingInterpolator.PiecewiseNode(
//                                        .8,
//                                        1.0,
//                                        HeadingInterpolator.linear(hp1.getHeading(), hp2.getHeading())
//                                )
//                        ))
//                .build();
//
//        path12End = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        hp1, hp2
//                ))
//                .setConstantHeadingInterpolation(hp2.getHeading())
//                .build();
//
//        path13 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        hp2, pose1
//                ))
//                .setHeadingInterpolation(
//                        HeadingInterpolator.piecewise(
//                                new HeadingInterpolator.PiecewiseNode(
//                                        0,
//                                        1.0,
//                                        HeadingInterpolator.tangent.reverse()
//                                )
//                        ))
//                .build();
//
//        leave = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pose1, new Pose(pose1.getX() + 5, pose1.getY())
//                ))
//                .setConstantHeadingInterpolation(H0)
//                .build();
//
    }

    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        vision = new AprilTagTracking(hardwareMap);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        super.reset();
        elapsedtime = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        elapsedtime.reset();

        chosenAlliance = "RED";
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        intake = new Intake(hardwareMap, telemetryM);
        shooter = new Shooter(hardwareMap, telemetryM);
        turret = new Turret(hardwareMap, telemetryM);
        turret.resetTurretEncoder();
        Shooter.landAngle = Math.toRadians(-15);

        intake.setAutoEnabled(true);
        shooter.setAutoShoot(true);
        turret.setAutoAim(false);
        Shooter.shooterDistanceBiasInches = 0;

        intake.setStopper(0.45);

        super.register(intake);
        super.register(shooter);
        super.register(turret);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Localization.init(follower, telemetryM);
        turret.setTurretPos(turret.getTargetTicksFromPos(startPose));

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
//                new stationary(follower),
//                new isAimed(turret),
                new slowTransfer(intake, true),
                new ParallelCommandGroup(
                        new WaitCommand(500),
                        new turretAutoAim(turret, true)
                ),
                new WaitCommand(250),
                new allBallsGone(intake).withTimeout(100),
                new slowTransfer(intake, false),
                new turretAutoAim(turret, false)
        );

        buildPaths();


        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                //Preload
//                new setTurretPos(turret, startPose),
                new shooterAtSpeed(shooter),
                shooterSequence,

                //3rd Mark
                new FollowPathCommand(follower, path10),
                shooterSequence,

                //Human Player
                new FollowPathCommand(follower, path4).withTimeout(1300),
                new ParallelRaceGroup(
                        new WaitCommand(85),
                        new allBallsDetected(intake)
                ),
                new ParallelCommandGroup(
                        new setTurretPos(turret, path5.endPose()),
                        new FollowPathCommand(follower, path5)
                ),
//                new FollowPathCommand(follower, path3),
                shooterSequence,

                //Human Player
                new FollowPathCommand(follower, path4).withTimeout(1300),
                new ParallelRaceGroup(
                        new WaitCommand(85),
                        new allBallsDetected(intake)
                ),

                new ParallelCommandGroup(
                        new setTurretPos(turret, path3.endPose()),
                        new FollowPathCommand(follower, path5)
                ),
//                new FollowPathCommand(follower, path3),
                shooterSequence,

                //Human Player


//Human Player
                new FollowPathCommand(follower, path2),
                new ParallelRaceGroup(
                        new WaitCommand(85),
                        new allBallsDetected(intake)
                ),

                new ParallelCommandGroup(
                        new setTurretPos(turret, path5.endPose()),
                        new FollowPathCommand(follower, path3)
                ),
//                new FollowPathCommand(follower, path3),
                shooterSequence,

                new FollowPathCommand(follower, path4Drifted).withTimeout(1300),
                new ParallelRaceGroup(
                        new WaitCommand(100),
                        new allBallsDetected(intake)
                ),


                new ParallelCommandGroup(
                        new setTurretPos(turret, path5.endPose()),
                        new FollowPathCommand(follower, path5Drifted)
                ),
//                new FollowPathCommand(follower, path3),
                shooterSequence,

                new FollowPathCommand(follower, path4Drifted).withTimeout(1300),
                new ParallelRaceGroup(
                        new WaitCommand(85),
                        new allBallsDetected(intake)
                ),


                new ParallelCommandGroup(
                        new setTurretPos(turret, path5.endPose()),
                        new FollowPathCommand(follower, path5Drifted)
                ),
//                new FollowPathCommand(follower, path3),
                shooterSequence,


//Human Player
                new FollowPathCommand(follower, path4)
//                new ParallelRaceGroup(
//                        new WaitCommand(500),
//                        new allBallsDetected(intake)
//                ),
//
//                new setTurretPos(turret, path3.endPose()),
//                new FollowPathCommand(follower, path3),
////                new FollowPathCommand(follower, path3),
//                new WaitCommand(200),
//                shooterSequence

//                //Collect Overflow
//                new FollowPathCommand(follower, collectOverflowRamp),
//                new WaitCommand(1000),
//
//                new setTurretPos(turret, overflowShoot.endPose()),
//                new FollowPathCommand(follower, overflowShoot),
////                new FollowPathCommand(follower, path3),
//                new WaitCommand(100),
//                shooterSequence,
//
//                //Collect Overflow
//                new FollowPathCommand(follower, collectOverflowRamp),
//                new WaitCommand(1000),
//
//                new setTurretPos(turret, overflowShoot.endPose()),
//                new FollowPathCommand(follower, overflowShoot),
////                new FollowPathCommand(follower, path3),
//                new WaitCommand(100),
//                shooterSequence,
//
//                //Collect Overflow
//                new FollowPathCommand(follower, collectOverflowRamp),
//                new WaitCommand(1000),
//
//                new setTurretPos(turret, overflowShoot.endPose()),
//                new FollowPathCommand(follower, overflowShoot),
////                new FollowPathCommand(follower, path3),
//                new WaitCommand(100),
//                shooterSequence




//                new FollowPathCommand(follower, path8),
//                new setTurretPos(turret, path9.endPose()),
//                new WaitCommand(750),
//                new FollowPathCommand(follower, path9),
////                new WaitCommand(200),
//                shooterSequence,
//                new FollowPathCommand(follower, path4, true),
//                new setTurretPos(turret, path5.endPose()),
//                new ParallelRaceGroup(
//                        new allBallsDetected(intake),
//                        new WaitCommand(1700)
//                ),
//                new FollowPathCommand(follower, path5),
////                new WaitCommand(200),
//                shooterSequence,
//                new setTurretPos(turret, path6.endPose()),
//                new FollowPathCommand(follower, path6),
////                new FollowPathCommand(follower, path7),
////                new WaitCommand(200),
//                shooterSequence,
//                new FollowPathCommand(follower, path8Drifted, true),
//                new setTurretPos(turret, path9.endPose()),
//                new WaitCommand(1000),
//                new FollowPathCommand(follower, path9),
////                new WaitCommand(200),
//                shooterSequence,
////                new FollowPathCommand(follower, path8),
////                new WaitCommand(1000),
////                new FollowPathCommand(follower, path9),
//////                new WaitCommand(200),
////                shooterSequence,
//                new setTurretPos(turret, path10.endPose()),
//                new FollowPathCommand(follower, path10),
////        new FollowPathCommand(follower, path11),
////        new WaitCommand(200),
//                shooterSequence
////new transfer(intake, true)
////                new FollowPathCommand(follower, leave)
////                new FollowPathCommand
////                        (follower, path12),
////                new FollowPathCommand(follower, path13, false),
////                shooterSequence
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
//        matchTimer.start();
        Localization.update();
        super.run();
        loopCounter += 1;

//        if(matchTimer.isLessThan(0.25) && (follower.getHeading() > Math.toRadians(4))) {
//            follower.holdPoint(follower.getPose());
////            follower.startTeleOpDrive(true);
////            follower.setTeleOpDrive(0, 0, 0);
//        }

        telemetry.addData("Actual Speed", 0.5*(shooter.getVelA() - shooter.getVelB()));
        telemetry.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);
        telemetry.update();
    }

    @Override
    public void end() {
        RobotConstants.savedPose = follower.getPose();
    }
}
