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
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;
//import com.skeletonarmy.marrow.TimerEx;

//import org.firstinspires.ftc.teamcode.commands.allBallsDetected;
//import org.firstinspires.ftc.teamcode.commands.allBallsGone;
import org.firstinspires.ftc.teamcode.commands.autoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.closeStopperCommand;
//import org.firstinspires.ftc.teamcode.commands.intakeOn1Command;
//import org.firstinspires.ftc.teamcode.commands.isAimed;
import org.firstinspires.ftc.teamcode.commands.openStopperCommand;
import org.firstinspires.ftc.teamcode.commands.powerFullIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.safeAllBallsDetected;
//import org.firstinspires.ftc.teamcode.commands.setShooterFromPose;
//import org.firstinspires.ftc.teamcode.commands.setTurretPos;
//import org.firstinspires.ftc.teamcode.commands.shooterAtSpeed;
//import org.firstinspires.ftc.teamcode.commands.slowTransfer;
//import org.firstinspires.ftc.teamcode.commands.stationary;
//import org.firstinspires.ftc.teamcode.commands.transfer;
//import org.firstinspires.ftc.teamcode.commands.turretAutoAim;
import org.firstinspires.ftc.teamcode.commands.shooterAtSpeedCommand;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
//import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@Autonomous(name = "Red Far Side 27 Path Auto")
public class RedFarAuto extends CommandOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private static final double H0 = Math.toRadians(0);

    List<LynxModule> allHubs;
    private static final double H30 = Math.toRadians(30);
    private static final double H60 = Math.toRadians(60);
    private static final double H80 = Math.toRadians(80);
    private static final double H_NEG20 = Math.toRadians(-20);


    private final Pose startPose = new Pose(86.1, 6.74, Math.toRadians(0));
    private final Pose pose1 = new Pose(86.1, 16, Math.toRadians(0));
    private final Pose humanPlayerRight = new Pose(132, 7.8, Math.toRadians(0));

//    private final Pose humanPlayerRightForward = new Pose(133, 50, Math.toRadians(40));
////    private final Pose nearMark = new Pose(120.128, 83.66);
////    private final Pose nearShoot = new Pose(94.608, 83.138);
////    private final Pose nearShootCP = new Pose(97.35, 66.32);
////    private final Pose collectMiddleMarkCP = new Pose(96.025, 62.109);
////    private final Pose collectMiddleMark = new Pose(126, 62.145);
//
//    private final Pose humanPlayerRightFirst = new Pose(136.3, 4.8, Math.toRadians(0));
//
//    private final Pose humanPlayerRightFirstDrifted = new Pose(138.3, 1.8, Math.toRadians(0));
    private final Pose collectLastMarkCP = new Pose(109.430, 37.845, Math.toRadians(-60));
    private final Pose collectLastMark = new Pose(128.360, 35.231, Math.toRadians(0));

//    private final Pose collectOverflow = new Pose(130.5, 30.1, Math.toRadians(40));

    private final Pose humanPlayerRightSlightBack = new Pose(110, 7.8, Math.toRadians(0));
    private final Pose poseAfterMiss = new Pose(110, 35, Math.toRadians(0));
    private final Pose poseAfterMissForward = new Pose(110, 35, Math.toRadians(0));
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
                                        HeadingInterpolator.linear(Math.toRadians(0), Math.toRadians(0))
                                )
                        )
                )
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        humanPlayerRight,
                        pose1
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
                                        HeadingInterpolator.constant(pose1.getHeading())
                                )
                        )
                )
                .build();



        path4 = follower.pathBuilder()
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

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        humanPlayerRight,
                        humanPlayerRightSlightBack
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
                                        HeadingInterpolator.constant(humanPlayerRightSlightBack.getHeading())
                                )
                        )
                )

                .addPath(new BezierCurve(
                        humanPlayerRightSlightBack,
                        poseAfterMiss,
                        poseAfterMissForward
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.constant(humanPlayerRightSlightBack.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(poseAfterMissForward.getHeading())
                                )
                        )
                )

                .addPath(new BezierLine(
                        poseAfterMissForward,
                        pose1
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.constant(poseAfterMissForward.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(pose1.getHeading())
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
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);


        intake.openStopper(false);

        super.register(intake);
        super.register(shooter);
        super.register(turret);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Localization.init(follower);
        turret.periodic();

        SequentialCommandGroup shooterSequence = new SequentialCommandGroup(
//                new stationary(follower),
//                new isAimed(turret),
                new openStopperCommand(intake),
                new powerFullIntakeCommand(intake),



                new WaitCommand(600),
                new closeStopperCommand(intake),
                new autoIntakeCommand(intake)
        );

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, path1),
                new shooterAtSpeedCommand(shooter),
                shooterSequence,

                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path2),
                        new autoIntakeCommand(intake)
                ),

                // AFTER path2 finishes, choose path3 or path4
                new ConditionalCommand(
                        new FollowPathCommand(follower, path3),
                        new FollowPathCommand(follower, path4),
                        () -> intake.all3
                )


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

        telemetry.addData("Actual Speed", 0.5*(shooter.shooterLeft.getVelocity()) - shooter.shooterRight.getVelocity());
        telemetry.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);
        telemetry.update();
    }

    @Override
    public void end() {
        RobotConstants.savedPose = follower.getPose();
    }
}
