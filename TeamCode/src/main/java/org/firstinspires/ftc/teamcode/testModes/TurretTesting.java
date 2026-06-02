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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.globals.Localization;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.TelemetryGroup;

@Configurable
@TeleOp(name = "Turret Testing", group = "TeleOp")
public class TurretTesting extends OpMode {

    private TelemetryManager telemetryM;
    private TelemetryGroup telemetryGroup;
    private Turret turret;
    private Follower follower;
    private Pose startPose = new Pose(72, 72, Math.toRadians(90));
    public Pose goalPose;


    @Override
    public void init() {
        turret = new Turret(hardwareMap);

        follower = createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        Localization.init(follower);

        chosenAlliance = "RED";
        goalPose = chosenAlliance.equals("RED") ? redGoalPose : blueGoalPose;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryGroup = new TelemetryGroup(telemetry, telemetryM);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);

        telemetryGroup.addData("Turret Test OpMode:", "INITIALIZED");
        telemetryGroup.update();
    }

    @Override
    public void loop() {
        Localization.update();
        Pose robotPose = getPose();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        turret.periodic();

        double targetHeading = turret.calculateTargetHeading(robotPose, goalPose);
        telemetryGroup.addData("Robot Pose:", robotPose);
        telemetryGroup.addData("Calculate heading (in radians):", targetHeading);
        telemetryGroup.addData("Target servo position:", turret.headingToTurretPos(targetHeading));
        telemetryGroup.addData("Current servo position:", turret.getServoPosition());
        telemetryGroup.update();
    }


    @Override
    public void stop() {
    }
}
