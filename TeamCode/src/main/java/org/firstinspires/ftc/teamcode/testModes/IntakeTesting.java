package org.firstinspires.ftc.teamcode.testModes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.TelemetryGroup;

@Configurable
@TeleOp(name = "Intake Testing", group = "TeleOp")
public class IntakeTesting extends OpMode {

    private TelemetryManager telemetryM;
    private TelemetryGroup telemetryGroup;
    private Intake intake;
    private Follower follower;

    public double intakePower = 0.0;
    public double transferPower = 0.0;
    public double stopperPosition = 0.0;

    @Override
    public void init() {
        intake = new Intake(hardwareMap, telemetryM);
        follower = createFollower(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryGroup = new TelemetryGroup(telemetry, telemetryM);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);

        telemetryGroup.addData("Intake Test OpMode:", "INITIALIZED");
        telemetryGroup.update();
    }

    @Override
    public void loop() {
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

        if (gamepad1.x) {
            transferPower = 1.0;
        }
        if (gamepad1.y) {
            transferPower = 0.0;
        }

        if (gamepad1.dpadUpWasPressed()) {
            intakePower += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            intakePower -= 0.05;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            transferPower += 0.05;
        }
        if (gamepad1.dpadRightWasPressed()) {
            transferPower -= 0.05;
        }

        if (gamepad1.leftBumperWasPressed()) {
            stopperPosition += 0.05;
        }
        if (gamepad1.rightBumperWasPressed()) {
            stopperPosition -= 0.05;
        }

        intake.setStopper(stopperPosition);
        intake.powerTransfer(transferPower);
        intake.powerIntake(intakePower);

        telemetryGroup.addData("Target Intake Power:", intakePower);
        telemetryGroup.addData("Target Transfer Power:", transferPower);
        telemetryGroup.addData("Target Stopper Position:", stopperPosition);
        telemetryGroup.addData("Actual Intake Power:", intake.getIntakePower());
        telemetryGroup.addData("Actual Transfer Power:", intake.getTransferPower());
        telemetryGroup.addData("Actual Stopper Position:", intake.getStopperPosition());
        telemetryGroup.update();
    }


    @Override
    public void stop() {
    }
}
