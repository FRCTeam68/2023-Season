package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
//import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake implements Subsystem {


    public enum SystemState{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        TRANSITION_CONE,
        PLACING
    }

    public SystemState currentState = SystemState.IDLE;
    private SystemState wantedState = SystemState.IDLE;
    

    public boolean haveCube;
    public boolean haveCone;

    private double currentStateStartTime = 0;

    private TalonFX intakeMotor;

    private final PS4Controller controller;


    public Intake(PS4Controller controller){
        intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        
        intakeMotor.configPeakOutputForward(1);
        intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setNeutralMode(NeutralMode.Brake); 
        haveCone = false;
        haveCube = false;

        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch (currentState){
            default:
            case INTAKING_CONE:
                newState = handleManual();
                break;
            case TRANSITION_CONE:
                newState = handleManual();
                break;
            case INTAKING_CUBE:
                newState = handleManual();
                break;
            case PLACING:
                newState = handleManual();
                break;
            case IDLE_CUBE:
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
        // if (controller.getL1ButtonPressed())
        //     setWantedState(SystemState.INTAKING_CONE);
        // if (controller.getL1ButtonReleased())
        //     setWantedState(SystemState.IDLE);

        // if (controller.getL2ButtonPressed())
        //     setWantedState(SystemState.INTAKING_CUBE);
        // if (controller.getL2ButtonReleased())
        //     setWantedState(SystemState.IDLE);

        if (currentState == SystemState.INTAKING_CONE && getIntakeCurrent() > 200){
            haveCone = true;
        }
        if (currentState == SystemState.INTAKING_CUBE && getIntakeCurrent() > 100){
            haveCube = true;
        }
        
        if(controller.getR2ButtonPressed()){
            if(controller.getL1Button())
            setWantedState(SystemState.PLACING);

            if(controller.getL2Button())
            setWantedState(SystemState.IDLE_CUBE);

            else
            setWantedState(SystemState.PLACING);

            haveCone = false;
            haveCube = false;
        }
            
        if (controller.getR2ButtonReleased())
            setWantedState(SystemState.IDLE);


    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(currentState){

            case INTAKING_CONE:
                setIntakeSpeed(-1);
                break;
            case TRANSITION_CONE:
                setIntakeSpeed(-.5);
                break;
            case INTAKING_CUBE:
                setIntakeSpeed(1);
                break;
            case PLACING:
                setIntakeSpeed(.7);
                break;
            case IDLE_CUBE:
                setIntakeSpeed(-.4);
                break;
            default:
            case IDLE:
                setIntakeSpeed(0);
                break;
        }
    }


    private SystemState handleManual(){
        return wantedState;
    }
    public void setWantedState(SystemState wantedState) {
		this.wantedState = wantedState;
	}
    public SystemState getCurrState(){
        return currentState;
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }
    public double getIntakeCurrent(){
        return intakeMotor.getStatorCurrent();
    }
    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putNumber("Current Stator Current", getIntakeCurrent());
        SmartDashboard.putNumber("Supply Current", intakeMotor.getSupplyCurrent());
        SmartDashboard.putBoolean("HaveCube", haveCube);
        SmartDashboard.putBoolean("HaveCone", haveCone);
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


    public SystemState getCurrentState(){
        return currentState;
    }

}