package org.firstinspires.ftc.teamcode.tests;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.targetTicks;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.disengagePos;
import static org.firstinspires.ftc.teamcode.globals.RobotConstants.engagePos;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretPID;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
//@Disabled
@TeleOp(name = "Turret Auto Aim Test", group = "TeleOp")
public class TurretTeleopTestV1 extends OpMode {
    private Turret turret;
    private Follower follower;
    private TelemetryManager telemetry;
    private Intake intake;
    private Shooter shooter;
    public static double engageTestPos = 1;
    public static double disengageTestPos = 0.8;
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0002;
    public static boolean setPoint = true;
    public static int target = 100;


    @Override
    public void init() {
        // Initializes the static Pinpoint wrapper (sets Pinpoint.pinpoint)
//        new Pinpoint(hardwareMap, telemetry);

        // Grab the same localizer so we can call update() explicitly
//        pinpointLocalizer = hardwareMap.get(PinpointLocalizer.class, "pinpoint");

        turret = new Turret(hardwareMap, null);
        turret.setAutoAim(false);
        turret.resetTurretEncoder();
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(28,0,Math.toRadians(90)));
        telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        turret.resetTurretEncoder();
        Localization.init(follower, telemetry);

        telemetry.addLine("Initialized. Press START to enable auto-aim.");
        telemetry.addLine("Make sure goal pose headings are radians (Math.toRadians(90)).");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        turret.setTargetTicks(target);
//        engagePos = 0.4;
//        disengagePos = 0.4;
//        shooter.setTargetEPT(1000);
//        shooter.setAutoShoot(false);
//        shooter.periodic();

        turretPID.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
        shooter.setAutoShoot(true);
        shooter.periodic();

        if (gamepad1.a) turret.resetTurretEncoder();   // reset
        if (gamepad1.b) {                              // manual profiled move
            turret.setAutoAim(false);
            turret.setTargetTicks(target);             // change target from dashboard
        }
        if (gamepad1.x) turret.setAutoAim(true);       // auto-aim + profiled tracking
        if (gamepad1.y) {                              // center test
            turret.setAutoAim(false);
            turret.straight();
        }

        turret.periodic(); // REQUIRED: runs profile + PID

//        if (gamepad1.a) {
//            intake.intake1On();
//        }
//
//        if (gamepad1.x) {
//            intake.setStopper(0.35);
//        }
//
//        if (gamepad1.leftBumperWasPressed()) {
//            intake.setPTO(disengageTestPos);
//        }
//
//        if (gamepad1.rightBumperWasPressed()) {
//            intake.setPTO(engageTestPos);
//        }
//
//        if (gamepad1.b) {
//            intake.intakeOff();
//        }


        telemetry.addData("encoder", turret.getPos());
        telemetry.addData("targetTicks", targetTicks);
        telemetry.addData("autoAim", turret.autoAimEnabled);
        telemetry.addData("aimed", turret.isAimed());
        telemetry.addData("current", intake.getCurrent());
        telemetry.update();
    }


    @Override
    public void stop() {
        turret.setAutoAim(false);
    }
}
