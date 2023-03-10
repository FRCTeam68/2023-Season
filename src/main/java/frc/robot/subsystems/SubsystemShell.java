package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

public class SubsystemShell implements Subsystem {
    // Shell for any subsystem class you might want to make. Please dont delete because that would be sad.
    private enum SystemState{
        EMPTY
    }

    public enum WantedState{
       EMPTY
    }

    private SystemState currentState = SystemState.EMPTY;
    private WantedState wantedState = WantedState.EMPTY;

    private double currentStateStartTime = 0;

    private final XboxController controller;

    public SubsystemShell(XboxController controller){

        this.controller = controller;

    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch(currentState){
            default:
                newState = SystemState.EMPTY;
                break;
        }

        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (currentState){
            default:
                break;
        }
    }

    @Override
    public void stop() {
        zeroSensors();
    }

    private SystemState handleManual(){
        switch (wantedState){
            default:
                return SystemState.EMPTY;

        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        
    }

    @Override
    public String getId() {
        return null;
    }
    
}
