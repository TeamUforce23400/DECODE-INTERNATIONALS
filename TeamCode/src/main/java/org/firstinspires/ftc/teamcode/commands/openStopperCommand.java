package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class openStopperCommand extends CommandBase {


    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public openStopperCommand(Intake subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.openStopper(true);
    }

    @Override
    public boolean isFinished() {
        return (intakeSubsystem.getStopperPosition() == intakeSubsystem.openPos);

    }
}
