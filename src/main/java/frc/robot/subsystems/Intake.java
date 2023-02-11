package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double currentTime = 0, graceTime = 0.020;

    private boolean holding = false;
    private double pastCurrent = 0;
    private double currentCurrent = 0;

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
        currentTime = timestamp;
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
        
        wantedState = (Math.abs(controller.getRightX())>Constants.DEADBAND) ? WantedState.MANUAL_CONTROL : WantedState.IDLE;
                
    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){
            
            case MANUAL_CONTROL:
                setIntakeSpeed(controller.getRightX());
                holding = idleHold(timestamp);
                break;
            case IDLE:
                setIntakeSpeed(-0.09);
                // if (holding){
                //     intakeMotor.set(ControlMode.Current, 18);
                // }                
                break;
            default:
            
        }
    }

    public boolean idleHold(double timestamp){
        if ((timestamp - currentStateStartTime) < graceTime)
            return false;

        if (intakeMotor.getStatorCurrent() > 24){
            
            return true;
        }
        
        return false;
    }

    private SystemState defaultStateChange() {
		switch (wantedState){
            case IDLE:
                return SystemState.IDLE;
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
        // intakeMotor.set(ControlMode.Current, 18);

    }

    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putNumber("Bus Voltage", intakeMotor.getBusVoltage());
        SmartDashboard.putNumber("Motor Voltage", intakeMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Current Stator Current", intakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Motor Output Voltage", intakeMotor.getMotorOutputVoltage());
        SmartDashboard.putNumber("Motor Output Percentage", intakeMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Motor Sensor Velocity", intakeMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Motor Sensor Position", intakeMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Supply Current", intakeMotor.getSupplyCurrent());
        SmartDashboard.putString("Wanted State", wantedState.name());
        SmartDashboard.putString("Current State", currentState.name());
        SmartDashboard.putBoolean("HOldinge", holding);
        SmartDashboard.putNumber("Current Time", timestamp);
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
