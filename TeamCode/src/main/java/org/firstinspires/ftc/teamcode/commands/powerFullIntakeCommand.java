package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class powerFullIntakeCommand extends CommandBase {


    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public powerFullIntakeCommand(Intake subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.powerFullIntake(1.0);
    }

    @Override
    public boolean isFinished() {
        return (intakeSubsystem.isIntakeOn);

    }
}
