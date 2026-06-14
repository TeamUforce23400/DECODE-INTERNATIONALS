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


import org.firstinspires.ftc.teamcode.commands.autoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.closeStopperCommand;
import org.firstinspires.ftc.teamcode.commands.openStopperCommand;
import org.firstinspires.ftc.teamcode.commands.powerFullIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooterAtSpeedCommand;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

@Autonomous(name = "Red 21 Path Auto New")
public class Red24New extends CommandOpMode {

    private Follower follower;
    private TelemetryManager telemetryM;
    private Intake intake;
    private Turret turret;
    private Shooter shooter;

    private static final double H0 = Math.toRadians(0);
    List<LynxModule> allHubs;
    private static final double H30 = Math.toRadians(30);
    private static final double H60 = Math.toRadians(-50);
    private static final double H75 = Math.toRadians(-70);
    private static final double H90 = Math.toRadians(-90);
    private static final double H_NEG20 = Math.toRadians(-20);

    private final Pose startPose = new Pose(107, 133.1, H90);
    private final Pose firstMark = new Pose(104, 87, H60);
    private final Pose firstMarkTowardsGoal = new Pose(117.5, 82, H0);
    private final Pose firstShoot = new Pose(96, 80, H0);
    private final Pose postFirstShoot = new Pose(82, 70, H0);
    private final Pose secondMark = new Pose(94, 55, H60);
    private final Pose secondMarkTowardsGoal = new Pose(120, 55, H0);
    private final Pose secondShoot = new Pose(100, 92, H0);



    private final Pose collectRamp = new Pose(133, 58.5, Math.toRadians(10)); // acc 59.4 y
    private final Pose preRamp = new Pose (130, 44, Math.toRadians(10));

    private final Pose postRamp = new Pose (144, 50, Math.toRadians(-15));

    private int loopCounter;
    private ElapsedTime elapsedtime;

    private PathChain path0, path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path10Mid, path10End, path11, path12, path13, path12Mid, path12End, path8Drifted, path8Drifted2, humanPlayerToShoot, humanPlayerCollectPathFinish, leave;

    private void buildPaths() {
        path0 = follower.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        postFirstShoot
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .15,
                                        HeadingInterpolator.linear(H90, H90)
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .15,
                                        1.0,
                                        HeadingInterpolator.constant(H90)
                                )

                        )
                ).build();

        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        postFirstShoot,
                        firstMark,
                        firstMarkTowardsGoal
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .15,
                                        HeadingInterpolator.linear(postFirstShoot.getHeading(), firstMark.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .15,
                                        1.0,
                                        HeadingInterpolator.constant(firstMarkTowardsGoal.getHeading())
                                )

                        )

                ).addPath( new BezierLine(
                                firstMarkTowardsGoal,
                                postFirstShoot
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(postFirstShoot.getHeading())
                                )
                        )
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        postFirstShoot,
                        secondMarkTowardsGoal
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.linear(postFirstShoot.getHeading(), secondMarkTowardsGoal.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.7,
                                        1,
                                        HeadingInterpolator.constant(secondMarkTowardsGoal.getHeading())
                                )

                        ))
                .addPath( new BezierLine(
                                secondMarkTowardsGoal,
                                firstShoot
                        )
                ).setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .7,
                                        1.0,
                                        HeadingInterpolator.constant(firstShoot.getHeading())
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

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        postFirstShoot,
                        new Pose(collectRamp.getX(), collectRamp.getY() - 0.8, collectRamp.getHeading())

                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .6,
                                        HeadingInterpolator.tangent
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .6,
                                        1.0,
                                        HeadingInterpolator.constant(collectRamp.getHeading()))
                        )
                )
                .build();
//
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        collectRamp,
//                        preRamp,
                        postRamp
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.linear(collectRamp.getHeading(), postRamp.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .75,
                                        1.0,
                                        HeadingInterpolator.constant(postRamp.getHeading()))
                        )
                )
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        postRamp,
                        postFirstShoot
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .75,
                                        1.0,
                                        HeadingInterpolator.constant(postFirstShoot.getHeading()))
                        )
                )
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        collectRamp,
//                        preRamp,
                        new Pose(postRamp.getX(), collectRamp.getY() - 0.1, collectRamp.getHeading())

                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.linear(collectRamp.getHeading(), postRamp.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .75,
                                        1.0,
                                        HeadingInterpolator.constant(postRamp.getHeading()))
                        )
                )
                .build();
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        collectRamp,
                        new Pose(postRamp.getX(), collectRamp.getY() + 1.6, collectRamp.getHeading())

                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        .7,
                                        HeadingInterpolator.linear(collectRamp.getHeading(), postRamp.getHeading())
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        .75,
                                        1.0,
                                        HeadingInterpolator.constant(postRamp.getHeading()))
                        )
                )
                .build();


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
        RobotConstants.redGoalPose = new Pose(147, 145, Math.toRadians(90));

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
                new FollowPathCommand(follower, path0)   ,
//            new shooterAtSpeedCommand(shooter),

                shooterSequence,
                new FollowPathCommand(follower, path2),
                shooterSequence,

                new FollowPathCommand(follower, path4),
                new WaitCommand(550),
                new FollowPathCommand(follower, path5),
                new FollowPathCommand(follower, path6),
                shooterSequence,

                new FollowPathCommand(follower, path4),
                new WaitCommand(550),
                new FollowPathCommand(follower, path5),
                new FollowPathCommand(follower, path6),
                shooterSequence,


                new ParallelCommandGroup(
                        new FollowPathCommand(follower, path1),
                        new closeStopperCommand(intake)
                ),
                shooterSequence,





                new FollowPathCommand(follower, path4),
                new WaitCommand(550),
                new FollowPathCommand(follower, path7),
                new FollowPathCommand(follower, path6),
                shooterSequence,

                new FollowPathCommand(follower, path4),
                new WaitCommand(550),
                new FollowPathCommand(follower, path7),
                new FollowPathCommand(follower, path6),
                shooterSequence,

                new FollowPathCommand(follower, path4),
                new WaitCommand(550),
                new FollowPathCommand(follower, path7),
                new FollowPathCommand(follower, path6),
                shooterSequence


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
