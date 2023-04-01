package frc.robot.subsystems;


import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
// import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Arm implements Subsystem {

    public enum SystemState{
        NEUTRAL,
        GROUND_CONE_ANGLE,
        GROUND_CUBE_ANGLE,
        HUMAN_FEED_CONE,
        HUMAN_FEED_CUBE,
        RAMP_FEED_CONE,
        RAMP_FEED_CUBE,
        CONE_HIGH,
        CONE_MID,
        CONE_GROUND,
        CUBE_HIGH,
        CUBE_MID,
        CUBE_GROUND,
        AUTON_CONE_MID,
        AUTON_CONE_HIGH,
        AUTON_CUBE_MID,
        AUTON_CUBE_HIGH,
        CONE_TRANSITION,
        CUBE_TRANSITION,
        MANUAL
    }

    private SystemState m_currentState = SystemState.NEUTRAL;
    private SystemState m_wantedState = SystemState.NEUTRAL;

    protected TalonFX m_extendMotor;
    protected MotionMagicVoltage m_extendMotorMMV;
    // protected CANcoder m_extendEncoder;
    protected DigitalInput m_extendLimitSwitch;

    protected TalonFX m_rotateMotor;
    protected MotionMagicVoltage m_rotateMotorMMV;
    protected CANcoder m_rotateEncoder;
    protected DigitalInput m_rotateLimitSwitch;
    protected VoltageOut m_rotateVoltageOut;

    protected double m_rotate_angle;
    protected double m_rotate_rotations;
    protected double m_extend_rotations;
    protected boolean m_extendLimitSwitchHit = false;
    // protected double m_extendLimitSwitch_debonce = 0;
    protected boolean m_rotateLimitSwitchHit = false;
    // protected double m_rotateLimitSwitch_debonce = 0;
    // protected boolean m_rotateSwitchStuck = false;
	
	protected double m_rotateEncoderAngle = 0;
    protected double m_rotateEncoderVelocity = 0;
    protected double m_rotateEncoderRotations = 0;

    protected TalonFX wristMotor;
    protected MotionMagicVoltage wristMotorMMV;
	protected double m_wrist_rotations=0;


    private final Intake m_intake;
    private final PS4Controller m_controller;
    private final PowerDistribution m_PDH; 

    private boolean m_manualMode = false;

    public Arm(PS4Controller controller, Intake intake){

        m_intake = intake;
        m_controller = controller;

        m_PDH = new PowerDistribution();

        // extendEncoderInit();
		extendMotorInit();
        m_extendLimitSwitch = new DigitalInput(9);

        //---------------------------------------------------------------------
        rotateEncoderInit();
        rotateMotorInit();
		m_rotateLimitSwitch = new DigitalInput(0);

        wristMotorInit();

    }

    private void extendMotorInit(){
        m_extendMotor = new TalonFX(Constants.ARM.EXTENDMOTOR, "MANIPbus");
        m_extendMotorMMV = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   
	
        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.0F; // Approximately 0.25V to get the mechanism moving

        // cfg.Feedback.SensorToMechanismRatio = 2F;
    
		//TODO make this like 0.5 percentoutput, so maxvoltage/2
        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // m_extendMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        // cfg.Voltage.PeakForwardVoltage = 4.0;
        // cfg.Voltage.PeakForwardVoltage = -4.0;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_extendMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure extend motor. Error: " + status.toString());
        }

        m_extendMotorMMV.OverrideBrakeDurNeutral = true;
        m_extendMotor.setVoltage(0);
        // m_extendMotor.setSafetyEnabled(false);

        zeroExtendSensor();
    }

	private void rotateMotorInit(){
        m_rotateMotor = new TalonFX(Constants.ARM.ROTATEMOTOR, "MANIPbus");
        m_rotateMotorMMV = new MotionMagicVoltage(0);  

        // for manual control we need to use voltageout
        m_rotateVoltageOut = new VoltageOut(0).withOverrideBrakeDurNeutral(true);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   

		m_rotateMotor.setInverted(true);
        
        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving
    
        
        // // tie CANcode on arm rotate shaft to the left motor
        // cfg.Feedback.FeedbackRemoteSensorID = m_rotateEncoder.getDeviceID();
        // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // // cfg.Feedback.SensorToMechanismRatio = 2F;
        
        // cfg.Voltage.PeakForwardVoltage = 4.0;
        // cfg.Voltage.PeakForwardVoltage = -4.0;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_rotateMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate motor. Error: " + status.toString());
        }

        m_rotateMotorMMV.OverrideBrakeDurNeutral = true;
        m_rotateMotor.setVoltage(0);
        // m_rotateMotor.setSafetyEnabled(false);

        zeroRotateSensor(); 
    }

    private void rotateEncoderInit(){
        m_rotateEncoder = new CANcoder(Constants.ARM.ROTATEENCODER, "MANIPbus");

        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    
        // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // cc_cfg.MagnetSensor.MagnetOffset = 0.1; //0; // 3.24; //0; //-2.82; //-1.82; //-1.2; //-1.52; //-1.77; // -1.866; //-1.74; //-1.82;
        m_rotateEncoder.getConfigurator().apply(cc_cfg);

        /* Speed up signals to an appropriate rate */
        // m_rotateEncoder.getPosition().setUpdateFrequency(100);
        // m_rotateEncoder.getVelocity().setUpdateFrequency(100);

        zeroRotateEncoder();
    }
	
    private void wristMotorInit(){
        wristMotor = new TalonFX(Constants.INTAKE.WRIST_MOTOR, "MANIPbus");
        wristMotorMMV = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotionMagic.MotionMagicCruiseVelocity = 2; //5
        cfg.MotionMagic.MotionMagicAcceleration = 2;

        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = wristMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate motor. Error: " + status.toString());
        }

        wristMotorMMV.OverrideBrakeDurNeutral = true;
        wristMotor.setVoltage(0);

        zeroWristSensor();
    }

    
    @Override
    public void processLoop(double timestamp) {
        
         SystemState newState;
         switch(m_currentState){
             default:
             case NEUTRAL:
                 newState = handleManual();
                 break;
             case GROUND_CONE_ANGLE:
                 newState = handleManual();
                 break;
             case GROUND_CUBE_ANGLE:
                 newState = handleManual();
                 break;
             case HUMAN_FEED_CONE:
                newState = handleManual();
                break;
            case HUMAN_FEED_CUBE:
                newState = handleManual();
                break;
            case CONE_HIGH:
                newState = handleManual();
                break;
            case CONE_MID:
                newState = handleManual();
                break;
            case CONE_GROUND:
                newState = handleManual();
                break;
            case CUBE_HIGH:
                newState = handleManual();
                break;
            case CUBE_MID:
                newState = handleManual();
                break;
            case CUBE_GROUND:
                newState = handleManual();
                break;
            case AUTON_CONE_HIGH:
                newState = handleManual();
                break;
            case AUTON_CONE_MID:
                newState = handleManual();
                break;
            case AUTON_CUBE_HIGH:
                newState = handleManual();
                break;
            case AUTON_CUBE_MID:
                newState = handleManual();
                break;
            case CONE_TRANSITION:
                newState = handleManual();
                break;
            case CUBE_TRANSITION:
                newState = handleManual();
                break;
            case  MANUAL:
                newState = handleManual();
                break;
         }

        if (m_wantedState != m_currentState) {
			m_currentState = newState;
        }
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
  

        m_extendLimitSwitchHit = m_extendLimitSwitch.get();
        m_rotateLimitSwitchHit = m_rotateLimitSwitch.get();

        m_rotateEncoderAngle = m_rotateEncoder.getAbsolutePosition().getValue();
        m_rotateEncoderVelocity = m_rotateEncoder.getVelocity().getValue();

        m_rotateEncoderRotations = m_rotateEncoderAngle*113.78;
		
		m_wrist_rotations=wristMotor.getPosition().getValue();

       if(!m_extendLimitSwitchHit){
           zeroExtendSensor();
       }

    //    if(!m_rotateLimitSwitchHit){
    //         //limit switch hit, position rotor should be at this point
    //         //TODO - should this be -1.26?  or this is hit going backwards, so stay positive.
    //         //  also should we be doing this every loop? and every time passed?
    //         //  does this mess-up motion magic?
    //         //  maybe should be only in manual mode?
            
    //         if (m_rotateLimitSwitch_debonce < 5 ){
    //             m_rotateLimitSwitch_debonce++;

    //         } else if (m_rotateLimitSwitch_debonce == 5 ){
    //             //we have a pulse low for 5 counts ( 5 * 20ms = 100ms)    
    //             m_rotateMotor.setRotorPosition(2.6);
    //             m_rotateLimitSwitch_debonce = 0;
    //         } else {
    //             // must be stuck,  do nothing
    //             m_rotateSwitchStuck = true;
    //         }
    //     } else {
    //         //switch is open, so count back to zero and stay there while open
    //         if (m_rotateLimitSwitch_debonce > 0)
    //             m_rotateLimitSwitch_debonce--;
    //     }
    

        if (!(m_currentState == SystemState.MANUAL)){
            // if(m_intake.getIntakeCurrent()>=200 && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.PLACING && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.IDLE)
               // setWantedState(SystemState.NEUTRAL);
            if(m_controller.getL1Button()){ //m_intake.getCurrState() == Intake.SystemState.INTAKING_CONE
                if(m_controller.getCrossButtonPressed()){
                    setWantedState(SystemState.GROUND_CONE_ANGLE);
                    m_intake.setWantedState(Intake.SystemState.INTAKING_CONE);
                }
                if(m_controller.getR1ButtonPressed()){
                    setWantedState(SystemState.HUMAN_FEED_CONE);
                    m_intake.setWantedState(Intake.SystemState.INTAKING_CONE);
                }
                if (m_intake.haveCone){
                    m_intake.setWantedState(Intake.SystemState.IDLE);
                }
                if(m_controller.getCircleButtonPressed())
                    setWantedState(SystemState.CONE_MID);
                if(m_controller.getSquareButtonPressed())
                    setWantedState(SystemState.CONE_HIGH);
            }
            if(m_controller.getL2Button()){ //m_intake.getCurrState() == Intake.SystemState.INTAKING_CUBE
                if(m_controller.getCrossButtonPressed()){
                    setWantedState(SystemState.GROUND_CUBE_ANGLE);
                    m_intake.setWantedState(Intake.SystemState.INTAKING_CUBE);
                }
                if(m_controller.getR1ButtonPressed()){
                    setWantedState(SystemState.HUMAN_FEED_CUBE);
                    m_intake.setWantedState(Intake.SystemState.INTAKING_CUBE);
                }

                if(m_controller.getCircleButtonPressed())
                    setWantedState(SystemState.CUBE_MID);
                if(m_controller.getSquareButtonPressed())
                    setWantedState(SystemState.CUBE_HIGH);
            }
            if(m_intake.haveCone){
                if(m_controller.getCircleButtonPressed())
                    setWantedState(SystemState.CONE_GROUND);
                
            }
            if(m_intake.haveCube){
                
                if(m_controller.getCrossButtonPressed())
                    setWantedState(SystemState.CUBE_GROUND);
            }
            
            if(m_controller.getCrossButtonReleased())
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getR1ButtonReleased())
                setWantedState(SystemState.NEUTRAL);

            if(m_controller.getTriangleButtonPressed()){
                if (m_currentState == SystemState.CONE_HIGH)
                    new SequentialCommandGroup(new InstantCommand(() -> setWantedState(SystemState.CONE_TRANSITION)),
                        new WaitCommand(0.5),
                        new InstantCommand(() -> setWantedState(SystemState.NEUTRAL))).schedule();
                if (m_currentState == SystemState.CUBE_HIGH)
                        new SequentialCommandGroup(new InstantCommand(() -> setWantedState(SystemState.CUBE_TRANSITION)),
                            new WaitCommand(0.5),
                            new InstantCommand(() -> setWantedState(SystemState.NEUTRAL))).schedule();
                else
                    setWantedState(SystemState.NEUTRAL);
            }
            
        }

		if (m_controller.getOptionsButtonPressed())
        	syncRotateMotor();
			
        if (m_controller.getPSButtonPressed()){
            
            if (m_currentState == SystemState.MANUAL){
                setWantedState(SystemState.NEUTRAL);
                m_manualMode = false;
            }else{
                setWantedState(SystemState.MANUAL);
                m_manualMode= true;
            }
        }
    }

    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (m_currentState){
            case GROUND_CONE_ANGLE:
				//4096 ticks in a revolution
				configRotate(-42.393);  //-20.8);   //42.693
                // configRotateAngle(-110);
				configExtend(11.148);     //13.138
                configWrist(0.513);	    //0.513
                break;
             case GROUND_CUBE_ANGLE:			
				configRotate(-46.152);  // -10.2);  //-45.375
                // configRotateAngle(-45);   //TODO: tweak angle
                configExtend(0);    //15.7939453125
                configWrist(1.51);             //1.904
                break;
            case HUMAN_FEED_CONE:
				configRotate(9.877); // -9.6);   //9.87
                // configRotateAngle(45);   //TODO: tweak angle
				configExtend(44.697);  //61.5);  //
                configWrist(2.276);              // 2.76
                break;
            case HUMAN_FEED_CUBE:
	            configRotate(10.668); // 11.2);   //10.688
                // configRotateAngle(-45);   //TODO: tweak angle
                configExtend(24.557);             //24.557
                configWrist(2.777);               //2.777
                break;
            case CONE_HIGH:
				configRotate(17.217);    //(41320-4000)/4096
                // configRotateAngle(-45);   //TODO: tweak angle
				configExtend(64.58);  //62.5);  //27.41);     //112256/4096
                configWrist(1.600);
                break;
            case CONE_MID:
                configExtend(29.015); //29.015
                configRotate(16.217);
                configWrist(1.669);
                break;
            case CONE_GROUND:
                configExtend(0);
                configRotate(25.92);
                configWrist(1.669);
                break;
            case CUBE_HIGH:
                configExtend(57.54);
                configRotate(-20.16);
                configWrist(.904);
                break;
            case CUBE_MID:
                configExtend(34.07);
                configRotate(-20.16);
                configWrist(.338);
                break;
            case CUBE_GROUND:
                configExtend(0);
                configRotate(-20.16);
                configWrist(.904);
                break;
            case AUTON_CONE_HIGH:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case AUTON_CONE_MID:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case AUTON_CUBE_HIGH:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case AUTON_CUBE_MID:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case CONE_TRANSITION:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case CUBE_TRANSITION:
                configExtend(0);
                configRotate(0);
                configWrist(0);
                break;
            case MANUAL:
                manualControl(m_controller.getLeftX(), -m_controller.getRightY());
                break;
            default:
            case NEUTRAL:
                // neutralize();
                configRotate(0);
                // configRotateAngle(0);
                configExtend(0);
                configWrist(2.45); //2.45
                break;
            
        }
    }

    private void syncRotateMotor(){
        // only do this when in a known location (i.e. not while arm moving)
        // And only do this if not too far off (because too far off suggests bad encoder)
        // if (Math.abs(m_rotateEncoderRotations-m_rotate_rotations) <2){
            m_rotateMotor.setRotorPosition(m_rotateEncoderRotations);
        // }
    }

    @Override
    public void stop() {
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putString("Arm State", m_currentState.toString());
        SmartDashboard.putBoolean("ExtendLimitSwitch", m_extendLimitSwitchHit);
        SmartDashboard.putBoolean("RotateLimitSwitch", m_rotateLimitSwitchHit);
        SmartDashboard.putNumber("Extend Motor Pos", m_extendMotor.getPosition().getValue());
        SmartDashboard.putNumber("Rotate Motor Pos", m_rotateMotor.getPosition().getValue());
        SmartDashboard.putNumber("Wrist Motor Pos", m_wrist_rotations);
		SmartDashboard.putNumber("Extend Motor Temp", m_extendMotor.getDeviceTemp().getValue());
		SmartDashboard.putNumber("Rotate Motor Temp", m_rotateMotor.getDeviceTemp().getValue());
        SmartDashboard.putNumber("Wrist Motor Temp", wristMotor.getDeviceTemp().getValue());
        // SmartDashboard.putNumber("rotate angle commanded", m_rotate_angle);
        SmartDashboard.putNumber("Rotate setpoint", m_rotate_rotations);
        SmartDashboard.putNumber("Extend setpoint", m_extend_rotations);
        SmartDashboard.putBoolean("Manual Mode", m_manualMode);
        SmartDashboard.putBoolean("L1 button", m_controller.getL1Button());
        SmartDashboard.putBoolean("L2 button", m_controller.getL2Button());
        // SmartDashboard.putString("rotate encoder pos", m_rotateEncoder.getPosition().toString());
        // SmartDashboard.putBoolean("Ext Motor sfty en", m_extendMotor.isSafetyEnabled());
        // SmartDashboard.putBoolean("Rot Motor sfty en", m_rotateMotor.isSafetyEnabled());
        SmartDashboard.putNumber("Ext Motor sup cur", m_extendMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Rot Motor sup cur", m_rotateMotor.getSupplyCurrent().getValue());
        SmartDashboard.putNumber("Arm Ecdr Angle", m_rotateEncoderAngle);
        SmartDashboard.putNumber("Arm Ecdr Rot", m_rotateEncoderRotations);
        SmartDashboard.putNumber("Arm Ecdr Velocity", m_rotateEncoderVelocity);
        SmartDashboard.putNumber("PDH Voltage", m_PDH.getVoltage());
        SmartDashboard.putNumber("PDH Current", m_PDH.getTotalCurrent());
    }

    private SystemState handleManual(){
        return m_wantedState;
    }

    public void setWantedState(SystemState wanted){
        m_wantedState = wanted;
    }

    public void configExtend(double position){
        //position is number of rotations, not number of ticks
        m_extend_rotations = position;
        m_extendMotor.setControl(m_extendMotorMMV.withPosition(position));
    }
   
    public void configRotate(double position){
        //position is number of rotations, not number of ticks
        m_rotate_rotations = position;
        m_rotateMotor.setControl(m_rotateMotorMMV.withPosition(position));
    }

    public void configWrist(double position){
        wristMotor.setControl(wristMotorMMV.withPosition(position));
    }

    // public void configRotateAngle(double angle){
    //     m_rotate_angle = angle;
    //     m_rotate_rotations = (m_rotate_angle) / 360;
    //     configRotate(m_rotate_rotations);
    // }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
	    zeroExtendSensor();
        zeroRotateSensor();
		zeroRotateEncoder();
        zeroWristSensor();
    }
   
    public void zeroExtendSensor(){
        m_extendMotor.setRotorPosition(0);
    }

    public void zeroRotateSensor(){
        m_rotateMotor.setRotorPosition(0);
        // m_rotateEncoder.setPosition(0);
        //move arm to vertical '0' position
        // configRotateAngle(0);
        //  No need to zero.   absolute CAN coder position will be used.
        //  so if it starts off zero, it will go to zero upon going to Nuetral state 
        //NOPE, magnet offset did not work.   go back to set rotor to zero.
    }

    public void zeroRotateEncoder(){
        m_rotateEncoder.setPosition(0);
    }
	
    public void zeroWristSensor(){
        wristMotor.setRotorPosition(0);
    }

   
    // private boolean getL1(){
    //     boolean triggered;
    //     if(m_controller.getL1ButtonPressed())
    //         triggered = true;
    //     if(m_controller.getL1ButtonReleased())
    //         triggered = false;

    //     return triggered;
    // }

    public void manualControl(double rotatePercentOutput, double armPercentOutput){
        m_rotateMotor.setControl(m_rotateVoltageOut.withOutput(Constants.ARM.MAX_MANUAL_SUPPLY_VOLTAGE*rotatePercentOutput/4));

        if (!m_extendLimitSwitch.get() && armPercentOutput < 0){
            m_extendMotor.setControl(m_rotateVoltageOut.withOutput(0));
        } else {
            m_extendMotor.setControl(m_rotateVoltageOut.withOutput(Constants.ARM.MAX_MANUAL_SUPPLY_VOLTAGE*armPercentOutput));
        }
    }


    @Override
    public String getId() {
        return "Arm";
    }

    
}