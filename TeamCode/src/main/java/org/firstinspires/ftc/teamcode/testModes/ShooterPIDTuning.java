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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryGroup;

import java.util.List;

@Configurable
@TeleOp(name = "Shooter PID Tuning", group = "TeleOp")
public class ShooterPIDTuning extends OpMode {

    private TelemetryManager telemetryM;
    private TelemetryGroup telemetryGroup;

    private Turret turret;
    private Shooter shooter;
    private Follower follower;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    public Pose goalPose;

    public static double targetVelocity = 0;
    public static double hoodPos = 0.7;

    public static boolean autoShooter = false;
    public static double P = 0;
    public static double I = 0;
    public static double kV = 0;
    public static double kS = 0;


    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryGroup = new TelemetryGroup(telemetry, telemetryM);

        allHubs = hardwareMap.getAll(LynxModule.class);

        shooter = new Shooter(hardwareMap);

        follower = createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Localization.init(follower);

        chosenAlliance = "RED";
        goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        shooter.hood.set(hoodPos);

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

        shooter.controllerLeft.setPIDF(P, I, 0, 0);
        shooter.controllerLeft.setFeedforward(kV, 0.0, kS);
        shooter.controllerRight.setPIDF(P, I, 0, 0);
        shooter.controllerRight.setFeedforward(kV, 0.0, kS);


        Localization.update();
        Pose robotPose = getPose();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        if (gamepad1.rightStickButtonWasReleased()) {
            autoShooter = !autoShooter;
        }

        if (gamepad1.dpadUpWasReleased()) {
            targetVelocity += 20;
        }

        if (gamepad1.dpadDownWasReleased()) {
            targetVelocity -= 20;
        }

        double shotDistanceMeters = robotPose.distanceFrom(goalPose) * 0.0254;
        double actualVelocityRight = shooter.shooterRight.getVelocity();
        double actualVelocityLeft = shooter.shooterLeft.getVelocity();

        if (autoShooter) {
            double[] coefficients = shooter.getCoefficientsFromDistance(shotDistanceMeters);
            targetVelocity = shooter.getTicksFromBallSpeed(coefficients[0]);
        }

        double errorRight = targetVelocity - shooter.shooterRight.getVelocity();
        double errorLeft = targetVelocity - shooter.shooterLeft.getVelocity();
        shooter.shooterRight.setPower(shooter.controllerRight.calculate(errorRight, targetVelocity, 0.0));
        shooter.shooterLeft.setPower(shooter.controllerLeft.calculate(errorLeft, targetVelocity, 0.0));


        telemetryGroup.addData("Robot Pose", robotPose);
        telemetryGroup.addData("Shot Distance M", shotDistanceMeters);

        telemetryGroup.addData("P", P);
        telemetryGroup.addData("I", I);
        telemetryGroup.addData("kV", kV);
        telemetryGroup.addData("kS", kS);

        telemetryGroup.addData("Auto Shooter", autoShooter);
        telemetryGroup.addData("Target Velocity", targetVelocity);
        telemetryGroup.addData("Right Velocity", actualVelocityRight);
        telemetryGroup.addData("Left Velocity", actualVelocityLeft);

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