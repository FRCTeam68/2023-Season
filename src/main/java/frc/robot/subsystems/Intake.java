package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Intake implements Subsystem {


    private enum SystemState{
        IDLE,   
        MANUAL_CONTROL,           //X,Y axis speeds relative to field
    }

    public enum WantedState{
        IDLE,
        MANUAL_CONTROL
    }

    private SystemState currentState = SystemState.MANUAL_CONTROL;
    private WantedState wantedState = WantedState.MANUAL_CONTROL;

    private double currentStateStartTime = 0;

    private TalonFX intakeMotor;

    private final XboxController controller;


    public Intake(XboxController controller){
        intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR);
        
        //intakeMotor.configPeakOutputForward(1);
        //intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setNeutralMode(Constants.INTAKE.INTAKE_NEUTRAL_MODE); 
        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch (currentState){
            default:
            case MANUAL_CONTROL:
                newState = handleManual();
                break;
            case IDLE:
                newState = handleManual();
                break;
      
        }
        if (newState != currentState) {
			currentState = newState;
			currentStateStartTime = timestamp;
		}
    }

    @Override
    public void readPeriodicInputs(double timestamp){
        /* 
        if(controller.getRightX()>=0.001){
            wantedState = WantedState.MANUAL_CONTROL;
        }
        else
        wantedState = WantedState.IDLE;
        */
    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){
            
            case MANUAL_CONTROL:
                setIntakeSpeed(controller.getRightX());
                break;
            default:
            case IDLE:
                setIntakeSpeed(0);
                break;
        }
    }

    private SystemState defaultStateChange() {
		switch (wantedState){
            /*case IDLE:
                return SystemState.IDLE;*/
            default:
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROL;
		}
	}

    private SystemState handleManual(){
        return defaultStateChange();
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void zeroSensors() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public String getId() {
        // TODO Auto-generated method stub
        return "Intake";
    }

    public void setWantedState(WantedState wantedState) {
		this.wantedState = wantedState;
	}
    
}
