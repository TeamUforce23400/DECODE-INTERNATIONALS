package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getGoalDistance;
import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.intakeBlueRamp;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.intakeRedRamp;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redPark;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redRampCP;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.savedPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@TeleOp(name = "Teleop Practice", group = "TeleOp")
public class TeleopPractice extends OpMode {
    private TelemetryManager telemetryM;
    private PIDFController controller1, controller2;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;

    private boolean newShooter = false;
    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private double yOffset = 0;
    private int speed = 0;
    private boolean autoAim = false;
    private double hoodPos = 0.7;
    private double sensitivity = 1;
    private ElapsedTime elapsedtime;
    private AprilTagTracking vision;
    private PolygonZone closeLaunchZone, farLaunchZone, robotZone;
    List<LynxModule> allHubs;
    private int loopCounter = 0;

    public void init() {
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        Shooter.landAngle = Math.toRadians(-15);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
        farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
        allHubs = hardwareMap.getAll(LynxModule.class);
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        shooter = new Shooter(hardwareMap, telemetryM);
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        robotZone = new PolygonZone(17, 17);
        vision = new AprilTagTracking(hardwareMap);
        Shooter.shooterDistanceBiasInches = 0;
        turret = new Turret(hardwareMap, telemetryM);
        turret.setAutoAim(true);
//        turret.resetTurretEncoder();
        intake = new Intake(hardwareMap, telemetryM);
        intake.setStopper(0.48);
        follower = createFollower(hardwareMap);
        if (savedPose != null && Math.abs(savedPose.getHeading()) > 1) {
            follower.setStartingPose(savedPose);
        } else {
//        follower.setStartingPose(savedPose != null ? savedPose : new Pose(88,75,Math.toRadians(0)));
            follower.setStartingPose(new Pose(104, 78, Math.toRadians(0)));
        }
        Localization.init(follower, telemetryM);
        telemetryM.addLine("Initialized");
        telemetry.addData("saved Pose:", savedPose);
        redGoalPose  = new Pose(142, 140, Math.toRadians(90));
        telemetry.update();
        telemetryM.update();
    }

    public void start() {
        follower.startTeleOpDrive(false);
        shooter.setHood(hoodPos);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
//        robotZone.setPosition(follower.getPose().getX(), follower.getPose().getY());
//        robotZone.setRotation(follower.getPose().getHeading());
        double shotDistance = follower.getPose().distanceFrom(redGoalPose);
        double actualShotSpeed = sh.getVelocity();

        if (shotDistance > 100) {
            Shooter.landAngle = Math.toRadians(-20);
            Shooter.powerConstant = 2.3;
        }
        else {
            Shooter.landAngle = Math.toRadians(-10);
            Shooter.powerConstant = 2.42;
        }

        double[] coefficients = Shooter.getCoefficientsFromDistance(shotDistance);
        targetVelocity = coefficients[1];
        if (Math.abs(actualShotSpeed - targetVelocity) > 40 && (shotDistance < 100)) {
            hoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, actualShotSpeed);
        } else {
            hoodPos = coefficients[0];
        }

        shooter.setHood(hoodPos);

        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
//
//        if (gamepad1.right_stick_x > 0.1 || gamepad1.left_stick_x > 0.1 || gamepad1.left_stick_y > 0.1) {
//            follower.breakFollowing();
//            follower.startTeleOpDrive(true);
//        }

        if (gamepad1.dpadDownWasReleased()) {
            redGoalPose  = new Pose(redGoalPose.getX(), redGoalPose.getY() - 1, Math.toRadians(90));
        }
        if (gamepad1.dpadUpWasReleased()) {
            redGoalPose  = new Pose(redGoalPose.getX(), redGoalPose.getY() + 1, Math.toRadians(90));
        }
        if (gamepad1.dpadLeftWasReleased()) {
            redGoalPose  = new Pose(redGoalPose.getX() + 1, redGoalPose.getY(), Math.toRadians(90));        }
        if (gamepad1.dpadRightWasReleased()) {
            redGoalPose  = new Pose(redGoalPose.getX() - 1, redGoalPose.getY(), Math.toRadians(90));        }

        if(intake.areAllBallsDetected()) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        if (gamepad1.right_trigger > 0.1) {
            sensitivity = 0.5;
        } else {
            sensitivity = 1;
        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.holdPoint(follower.getPose());
        }
        else if (gamepad1.rightBumperWasReleased()) {
            follower.startTeleOpDrive(false);
        }

        if (gamepad1.optionsWasPressed()) {
            follower.setPose(new Pose(4,9,Math.toRadians(90)));
        }


//        if (gamepad1.rightBumperWasPressed()) {
//            if ((follower.getPose().distanceFrom(intakeRedRamp) < 20))
//            {
//                follower.followPath(follower.pathBuilder()
//                        .addPath(new BezierCurve(
//                                follower.getPose(),
//                                redRampCP,
//                                intakeRedRamp
//                        ))
//                        .setConstantHeadingInterpolation(intakeRedRamp.getHeading())
//                        .build());
//            }
//            else {
//                follower.followPath(follower.pathBuilder()
//                        .addPath(new BezierCurve(
//                                follower.getPose(),
//                                redRampCP,
//                                intakeRedRamp
//                        ))
//                        .setHeadingInterpolation(
//                                HeadingInterpolator.piecewise(
//                                        new HeadingInterpolator.PiecewiseNode(
//                                                0,
//                                                .5,
//                                                HeadingInterpolator.constant(Math.toRadians(0))
//                                        ),
//                                        new HeadingInterpolator.PiecewiseNode(
//                                                .5,
//                                                1.0,
//                                                HeadingInterpolator.constant(intakeRedRamp.getHeading()))
//                                )
//                        )
//                        .build());
//            }
//        }
//        else if (gamepad1.rightBumperWasReleased()) {
//            follower.startTeleOpDrive(false);
//        }

        if (gamepad1.leftBumperWasPressed()) {
            follower.followPath(follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            new Pose(84, 75)
                    ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(
                                            0,
                                            1.0,
                                            HeadingInterpolator.tangent.reverse()
                                    )
                            ))
                    .build());
        }
        else if (gamepad1.leftBumperWasReleased()) {
            follower.startTeleOpDrive(false);
        }



        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * sensitivity,
                -gamepad1.left_stick_x * sensitivity,
                -gamepad1.right_stick_x * sensitivity,
                true
        );
        if (gamepad1.a) {
            shooter.setHood(hoodPos);
            intake.intake1On();
        }
        if (gamepad2.right_bumper) {
            intake.intake1On();
        }
        if (gamepad2.rightBumperWasReleased()) {
            intake.intakeOff();
        }

        if (gamepad2.left_bumper) {
            intake.engagePTO();
            intake.setStopper(0.35);
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.8);
            }
            else {
                intake.onSpeed(1);
            }
        }
        else if (gamepad2.leftBumperWasReleased()) {
            intake.intakeOff();
            intake.setStopper(0.48);
        }
//
//        if (robotZone.isInside(closeLaunchZone)) {
//            intake.setStopper(0.35);
//            intake.intake1On();
//        }
//
//        if (intake.areAllBallsDetected() && (!robotZone.isInside(closeLaunchZone) && !robotZone.isInside(farLaunchZone))) {
//            intake.intakeOff();
//        }
//
//        if (!robotZone.isInside(closeLaunchZone) && !robotZone.isInside(farLaunchZone)) {
//            intake.setStopper(0.45);
//        }

//        if (gamepad1.dpadLeftWasReleased()) {
//            hoodPos += 0.025;
//            shooter.setHood(hoodPos);
//        }





        if (sh.getVelocity() > targetVelocity) {
            sh.setPower(0);
            sh2.setPower(0);
        }
        else if (sh.getVelocity() < targetVelocity) {
            sh.setPower(1);
            sh2.setPower(1);
        }
        else {
            sh.setPower(0);
            sh2.setPower(0);
        }
//        sh.setPower(controller1.calculate(targetVelocity - velocity1, targetVelocity, 0.0));
//        sh2.setPower(controller2.calculate(targetVelocity - velocity2, targetVelocity, 0.0));

//        telemetryM.addData("Odometry pose", getPose());
//        telemetryM.addData("Limelight pose", vision.getLocalization());
//        telemetryM.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);

        loopCounter +=1;

        Localization.update();
        turret.periodic();
        intake.periodic();
        telemetry.update();
//        telemetryM.update();
    }
}
