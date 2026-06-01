package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Intake extends SubsystemBase {

    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    private final ServoEx stopper;

    public final double openPos = 0; // TODO: This will be your stopper open position.
    public final double closePos = 1; // TODO: This will be your stopper close position.

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        stopper = new ServoEx(hardwareMap, "servo");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
    }

    public void powerIntake(double power) {
        intake.setPower(power);
    }

    public void powerTransfer(double power) {
        transfer.setPower(power);
    }

    public void powerFullIntake(double power) {
        intake.setPower(power);
        transfer.setPower(power);
    }

    public void setStopper(double position) {
        stopper.set(position);
    }

    public void openStopper(boolean open) {
        if (open) {
            stopper.set(openPos);
        } else {
            stopper.set(closePos);
        }
    }

    public double getIntakePower() {
        return intake.getPower();
    }

    public double getTransferPower() {
        return transfer.getPower();
    }

    public double getStopperPosition() {
        return stopper.getRawPosition();
    }

}