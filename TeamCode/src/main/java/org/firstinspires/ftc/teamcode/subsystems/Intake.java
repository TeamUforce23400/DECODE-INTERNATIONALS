package org.firstinspires.ftc.teamcode.subsystems;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class Intake extends SubsystemBase {

    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    private final ServoEx stopperRight;
//    private final ServoEx stopperLeft;
    private final ServoExGroup stopper;

    public final double openPos = 0.1; // TODO: This will be your stopper open position.
    public final double closePos = 0.4; // TODO: This will be your stopper close position.

    public Intake(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        intake = hardwareMap.get(DcMotorEx.class, "inr");
        transfer = hardwareMap.get(DcMotorEx.class, "inl");

        stopperRight = new ServoEx(hardwareMap, "sr");
//        stopperLeft = new ServoEx(hardwareMap, "sl");
        stopper = new ServoExGroup(stopperRight);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotor.Direction.REVERSE);
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
        transfer.setPower(power);
        intake.setPower(power);
    }

    public void openStopper(boolean open) {
        if (open) {
            stopper.set(openPos);
        } else {
            stopper.set(closePos);
        }
    }

    public void setStopper(double position) {
        stopper.set(position);
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
