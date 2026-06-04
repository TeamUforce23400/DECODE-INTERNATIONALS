package org.firstinspires.ftc.teamcode.testModes;

import static org.firstinspires.ftc.teamcode.globals.Localization.getPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.blueGoalPose;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.chosenAlliance;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.redGoalPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryGroup;

import java.util.List;

@Configurable
@TeleOp(name = "Turret + Shooter Testing", group = "TeleOp")
public class ShooterTurretTest extends OpMode {

    private TelemetryManager telemetryM;
    private TelemetryGroup telemetryGroup;

    private Turret turret;
    private Shooter shooter;
    private Intake intake;

    private Follower follower;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    public Pose goalPose;

    public static double targetVelocity = 0;
    public static double hoodPos = 0.0;

    public static boolean autoShooter = false;

    public double intakePower = 0.0;
    public double transferPower = 0.0;

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryGroup = new TelemetryGroup(telemetry, telemetryM);

        allHubs = hardwareMap.getAll(LynxModule.class);

        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, telemetryM);

        follower = createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Localization.init(follower);

        chosenAlliance = "RED";
        goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

//        shooter.hood.set(hoodPos);

        targetVelocity = 0;
        autoShooter = false;

        telemetryGroup.addData("Status", "Initialized");
        telemetryGroup.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        Localization.update();
        Pose robotPose = getPose();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        if (gamepad1.a) {
            intakePower = 1.0;
        }
        if (gamepad1.b) {
            intakePower = 0;
        }

        if (gamepad1.rightStickButtonWasReleased()) {
            autoShooter = !autoShooter;
        }

        if (gamepad1.dpadUpWasReleased()) {
            targetVelocity = targetVelocity + 20;
        }

        if (gamepad1.dpadDownWasReleased()) {
            targetVelocity = targetVelocity - 20;
        }

        if (gamepad1.dpadRightWasReleased()) {
            hoodPos = Range.clip(hoodPos - 0.025, shooter.minimumHoodPos, shooter.maximumHoodPos);
            shooter.hood.set(hoodPos);
        }

        if (gamepad1.dpadLeftWasReleased()) {
            hoodPos = Range.clip(hoodPos + 0.025, shooter.minimumHoodPos, shooter.maximumHoodPos);
            shooter.hood.set(hoodPos);
        }

        double shotDistanceMeters = robotPose.distanceFrom(goalPose) * 0.0254;
        double actualVelocityRight = shooter.shooterRight.getVelocity();
        double actualVelocityLeft = shooter.shooterLeft.getVelocity();

        if (autoShooter) {
            double[] coefficients = shooter.getCoefficientsFromDistance(shotDistanceMeters);

            double calculatedTargetVelocity = shooter.getTicksFromBallSpeed(coefficients[0]);
            double calculatedHoodAngle = coefficients[1];

            targetVelocity = calculatedTargetVelocity;

            if (Math.abs(targetVelocity - actualVelocityRight) > 30) {
                calculatedHoodAngle = shooter.getCompensatedHoodAngle(
                        shotDistanceMeters,
                        actualVelocityRight
                );
            }

            hoodPos = Range.clip(
                    shooter.getHoodPosFromAngle(calculatedHoodAngle),
                    shooter.minimumHoodPos,
                    shooter.maximumHoodPos
            );

            shooter.hood.set(hoodPos);
        }

        if (targetVelocity <= 0) {
            shooter.shooterRight.setPower(0);
            shooter.shooterLeft.setPower(0);
        } else if (actualVelocityRight < targetVelocity - 10) {
            shooter.shooterRight.setPower(1);
            shooter.shooterLeft.setPower(1);
        } else {
            shooter.shooterRight.setPower(0);
            shooter.shooterLeft.setPower(0);
        }

        intake.powerTransfer(transferPower);
        intake.powerIntake(intakePower);

        turret.periodic();

        double targetHeading = turret.calculateTargetHeading(robotPose, goalPose);

        telemetryGroup.addData("Robot Pose", robotPose);
        telemetryGroup.addData("Shot Distance M", shotDistanceMeters);

        telemetryGroup.addData("Auto Shooter", autoShooter);
        telemetryGroup.addData("Target Velocity", targetVelocity);
        telemetryGroup.addData("Right Velocity", actualVelocityRight);
        telemetryGroup.addData("Left Velocity", actualVelocityLeft);
        telemetryGroup.addData("Hood Pos", hoodPos);
        telemetryGroup.addData("Raw Hood Pos", shooter);

        telemetryGroup.addData("Turret Target Heading Rad", targetHeading);
        telemetryGroup.addData("Target Servo Position", turret.headingToTurretPos(targetHeading));
        telemetryGroup.addData("Current Servo Position", turret.getServoPosition());

        telemetryGroup.update();
    }

    @Override
    public void stop() {
        targetVelocity = 0;
        autoShooter = false;

        shooter.shooterRight.setPower(0);
        shooter.shooterLeft.setPower(0);
    }
}