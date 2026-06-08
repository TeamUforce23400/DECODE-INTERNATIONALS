package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class autoIntakeCommand extends CommandBase {


    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake intakeSubsystem;


    public autoIntakeCommand(Intake subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.autoIntake();
    }

    @Override
    public boolean isFinished() {
        return (intakeSubsystem.all3);

    }
}
