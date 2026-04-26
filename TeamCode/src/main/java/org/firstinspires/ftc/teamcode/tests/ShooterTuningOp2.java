package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.globals.Localization.getHeading;
import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.Localization.getRedDistance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.farRedGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.maxEPT;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.resetPos;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.vision.AprilTagTracking;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter tuning v2", group = "TeleOp")
public class ShooterTuningOp2 extends OpMode {
    private TelemetryManager telemetryM;
    private PIDFController controller1, controller2;
    private DcMotorEx sh;
    private DcMotorEx sh2;
    public static double targetVelocity, velocity1, velocity2;
    public static double P,I,kV,kS;
    public double shooterCurrent;

    private boolean newShooter = false;
    private boolean activeHood = false;
    private Turret turret;
    private Follower follower;
    private Shooter shooter;
    private Intake intake;

    private int speed = 0;
    private boolean autoAim = false;
    private double hoodPos = 0.7;
    private ElapsedTime elapsedtime;
    private AprilTagTracking vision;
    List<LynxModule> allHubs;
    private int loopCounter = 0;

    public void init() {
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        sh = hardwareMap.get(DcMotorEx.class, "rsh");
        allHubs = hardwareMap.getAll(LynxModule.class);
        sh2 = hardwareMap.get(DcMotorEx.class, "lsm");
        shooter = new Shooter(hardwareMap, telemetryM);
        sh.setDirection(DcMotorSimple.Direction.FORWARD);
        sh2.setDirection(DcMotorSimple.Direction.FORWARD);
        vision = new AprilTagTracking(hardwareMap);
        targetVelocity = 0;
        velocity1 = 0;
        velocity2 = 0;
        autoAim = false;
        newShooter = false;
        I = 0.3;
        P = 1.5;
        kS = 0.05;
        kV = 0.00039;
        chosenAlliance = "RED";
        controller1 = new PIDFController(P,I,0.0, 0);
        controller2 = new PIDFController(P,I,0.0, 0);
        turret = new Turret(hardwareMap, telemetryM);
        Shooter.landAngle = Math.toRadians(-15);
        Shooter.shooterDistanceBiasInches = 0;
        redGoalPose  = new Pose(141, 141, Math.toRadians(90));
        turret.resetTurretEncoder();
        intake = new Intake(hardwareMap, telemetryM);
        sh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sh2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setStopper(0.48);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(135,9,Math.toRadians(90)));
        Localization.init(follower, telemetryM);

        telemetryM.addLine("Initialized");
        telemetryM.update();
    }

    public void start() {
        follower.startTeleOpDrive(true);
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
        double shotDistance = follower.getPose().distanceFrom(redGoalPose);
        double actualShotSpeed = sh.getVelocity();
        double compensatedHoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, actualShotSpeed);

        if (gamepad1.optionsWasPressed()) {
            follower.setPose(vision.getLocalization());
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if (gamepad1.a) {
//            hoodPos = compensatedHoodPos;
//            shooter.setHood(hoodPos);
            intake.intake1On();
        }
        if (gamepad1.right_bumper) {
            intake.setStopper(0.35);
            intake.engagePTO();
            activeHood = true;
            double farExtraInches = Math.max(0, getRedDistance() - 110);
            if(farExtraInches > 0) {
                intake.onSpeed(0.7);
            }
            else {
                intake.intake1On();
            }
        }

        if (gamepad1.rightBumperWasReleased()) {
            intake.setStopper(0.48);
            intake.intakeOff();
            activeHood = false;
        }
        if (gamepad1.b) {
            intake.intakeOff();
        }
        if (gamepad1.y) {
            turret.resetTurretEncoder();
        }
        shooterCurrent = sh.getCurrent(CurrentUnit.AMPS);
        if (gamepad1.right_trigger > 0.1) {
//            follower.holdPoint(follower.getPose());
            follower.setPose(resetPos);
        }
        if (gamepad1.leftTriggerWasPressed()) {
            follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(
                                follower.getPose(),
                                new Pose(follower.getPose().getX() + 1, follower.getPose().getY() + 1)
                        ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.facingPoint(redGoalPose)
                    )
                        .build());
        }
        else if (gamepad1.leftTriggerWasReleased()){
            follower.startTeleOpDrive(false);
        }

        if (gamepad1.leftStickButtonWasReleased()) {
            autoAim = !autoAim;
            turret.setAutoAim(autoAim);
        }
        if(gamepad1.dpadUpWasReleased()) {
            targetVelocity = Range.clip(targetVelocity + 20, 0, maxEPT);
        }
        if(gamepad1.dpadDownWasReleased()) {
            targetVelocity = Range.clip(targetVelocity - 20, 0, maxEPT);
        }
        if (gamepad1.dpadRightWasReleased()) {
            hoodPos -= 0.025;
            shooter.setHood(hoodPos);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hoodPos += 0.025;
            shooter.setHood(hoodPos);
        }

        if (gamepad1.rightStickButtonWasReleased()) {
            newShooter = !newShooter;
        }

        if (shotDistance > 100) {
            Shooter.landAngle = Math.toRadians(-20);
            Shooter.powerConstant = 2.3;
        }
        else {
            Shooter.landAngle = Math.toRadians(-10);
            Shooter.powerConstant = 2.42;
        }

        if (newShooter) {
            double[] coefficients = Shooter.getCoefficientsFromDistance(shotDistance);
            targetVelocity = Range.clip(coefficients[1], 0, maxEPT);
            if (Math.abs(actualShotSpeed - targetVelocity) > 40 && (shotDistance < 100)) {
                hoodPos = Shooter.getLowAngleHoodFromDistanceAndSpeed(shotDistance, sh.getVelocity());
            } else {
                hoodPos = coefficients[0];
            }
            shooter.setHood(hoodPos);

//            if (Math.max(0, getRedDistance() - 110)>0){
////                redGoalPose = farRedGoalPose;
////                targetVelocity += 110;
////                Shooter.landAngle = Math.toRadians(-25);
//            }
//
//            else {
////                redGoalPose = new Pose (141, 141, Math.toRadians(90));
//                Shooter.landAngle = Math.toRadians(-15);
//            }


        }


        velocity1 = sh.getVelocity();
        velocity2 = sh2.getVelocity();
        if (targetVelocity <= 0) {
            sh.setPower(0);
            sh2.setPower(0);
        }
        else if (actualShotSpeed < targetVelocity-10) {
            sh.setPower(1);
            sh2.setPower(1);
        }
        else {
            sh.setPower(0);
            sh2.setPower(0);
        }
//        sh.setPower(controller1.calculate(targetVelocity - velocity1, targetVelocity, 0.0));
//        sh2.setPower(controller2.calculate(targetVelocity - velocity2, targetVelocity, 0.0));

        Localization.update();
        turret.periodic();
        intake.periodic();

//        Turret.AimDebug aim = turret.getAimDebug();
        telemetryM.addData("shooter", targetVelocity);
        telemetryM.addData("actual shot speed", actualShotSpeed);
        telemetryM.addData("hood pos", hoodPos);
//        telemetryM.addData("shooter current", shooterCurrent);
//        telemetryM.addData("auto aim", autoAim);
//        telemetryM.addData("Odometry pose", getPose());
//        telemetryM.addData("aim alliance", aim.alliance);
//        telemetryM.addData("aim goal", aim.goalName);
//        telemetryM.addData("aim red dist", aim.redDistanceIn);
//        telemetryM.addData("aim robot x", aim.robotX);
//        telemetryM.addData("aim robot y", aim.robotY);
//        telemetryM.addData("aim robot heading deg", aim.robotHeadingDeg);
//        telemetryM.addData("aim turret forward offset", aim.turretForwardOffsetIn);
//        telemetryM.addData("aim turret left offset", aim.turretLeftOffsetIn);
//        telemetryM.addData("aim turret zero deg", aim.turretZeroOffsetDeg);
//        telemetryM.addData("aim turret ticks/deg", aim.turretTicksPerDegree);
//        telemetryM.addData("aim turret direction", aim.turretEncoderDirection);
//        telemetryM.addData("aim turret origin x", aim.turretOriginX);
//        telemetryM.addData("aim turret origin y", aim.turretOriginY);
//        telemetryM.addData("aim goal x", aim.goalX);
//        telemetryM.addData("aim goal y", aim.goalY);
//        telemetryM.addData("aim goal bearing deg", aim.goalBearingDeg);
//        telemetryM.addData("aim velocity lead deg", aim.velocityLeadDeg);
//        telemetryM.addData("aim target abs deg", aim.targetAbsHeadingDeg);
//        telemetryM.addData("aim rel raw deg", aim.relToTargetRawDeg);
//        telemetryM.addData("aim rel norm deg", aim.relToTargetNormDeg);
//        telemetryM.addData("aim chosen rel deg", aim.chosenRelDeg);
//        telemetryM.addData("aim current ticks", aim.currentTurretTicks);
//        telemetryM.addData("aim current rel deg", aim.currentTurretRelDeg);
//        telemetryM.addData("aim current abs deg", aim.currentTurretAbsDeg);
//        telemetryM.addData("aim target ticks raw", aim.targetTicksDouble);
//        telemetryM.addData("aim target ticks", aim.targetTicksInt);
//        telemetryM.addData("aim expected physical rel", aim.expectedPhysicalRelDeg);
//        telemetryM.addData("aim clipped", aim.clipped);
//        telemetryM.addData("aim pid power", aim.pidPower);
//        telemetry.addData("aim goal", aim.goalName);
//        telemetry.addData("aim heading", aim.robotHeadingDeg);
//        telemetry.addData("aim rel", aim.chosenRelDeg);
//        telemetry.addData("aim ticks", aim.targetTicksInt);
//        telemetry.addData("current", turret.getPos());
//        telemetryM.addData("Limelight pose", vision.getLocalization());
//        telemetryM.addData("Loop Times", elapsedtime.milliseconds()/loopCounter);
//        telemetry.addData("S3", intake.isBallDetected03());
//        telemetry.addData("S2", intake.isBallDetected02());
//        telemetry.addData("S1", intake.isBallDetected01());

        loopCounter +=1;

        telemetry.update();
        telemetryM.update();
    }

    @Override
    public void stop() {
        targetVelocity = 0;
        newShooter = false;
        autoAim = false;
        sh.setPower(0);
        sh2.setPower(0);
        intake.intakeOff();
        turret.setAutoAim(false);
    }
}
