package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeWantedStateCommand extends InstantCommand {
    
    private final Intake intake;
    private final Intake.SystemState wantedState;

    public IntakeWantedStateCommand(final Intake intake, Intake.SystemState wantedState){
        this.intake = intake;
        this.wantedState = wantedState;
    }

    @Override
    public void initialize(){
        intake.setWantedState(wantedState);
    }
}
