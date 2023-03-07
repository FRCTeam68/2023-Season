package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
// import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.controls.Follower;

import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
// import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.hardware.CANcoder;

// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;


public class Arm implements Subsystem {

    public enum SystemState{
        NEUTRAL,
        GROUND_ANGLE,
        HUMAN_FOLD,
        PLACING,
        HIGH,
        MID,
        TRAVEL,
        START;
    }

    private SystemState m_currentState = SystemState.NEUTRAL;
    private SystemState m_wantedState = SystemState.NEUTRAL;

    protected TalonFX m_liftMotor;
    protected MotionMagicVoltage m_liftMotorMMV;
    // protected CANcoder m_liftEncoder;
    protected DigitalInput m_liftLimitSwitch;

    protected TalonFX m_rotateMotorLeft;
    protected MotionMagicVoltage m_rotateMotorLeftMMV;
    protected TalonFX m_rotateMotorRight;
    protected MotionMagicVoltage m_rotateMotorRightMMV;
    protected CANcoder m_rotateEncoder;
    protected DigitalInput m_rotateLimitSwitch;

    // protected ElevatorFeedforward m_feedforward;
    // private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(10, 2);

    // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();

    // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private final Intake m_intake;
    private final PS4Controller m_controller;

    public Arm(PS4Controller controller, Intake intake){

        m_intake = intake;
        m_controller = controller;

        liftMotorInit();
        // liftEncoderInit();
        m_liftLimitSwitch = new DigitalInput(0);

        //---------------------------------------------------------------------
        rotateEncoderInit();
        rotateLeftMotorInit();
        rotateRightMotorInit();
        m_rotateMotorRight.setControl(new Follower(m_rotateMotorLeft.getDeviceID(), false));

        m_rotateLimitSwitch = new DigitalInput(1);
        //--------------------------------------------------------------------- 

        // feedforward = new ElevatorFeedforward(0.01, 0, 0.06);
        // liftMotor.setSensorPhase(true);
        // rotateMotorLeft.setSensorPhase(true);
        // rotateMotorRight.setSensorPhase(true);
    }

    private void liftMotorInit(){
        m_liftMotor = new TalonFX(Constants.Arm.LIFTMOTOR, "MANIPbus");
        m_liftMotorMMV = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 20; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 40; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 50;   


        cfg.Slot0.kP = 4.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.0F; // Approximately 0.25V to get the mechanism moving

        // cfg.Feedback.SensorToMechanismRatio = 2F;
    
        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V
        // liftMotor.configPeakOutputForward(0.2);
        // liftMotor.configPeakOutputReverse(-0.2);

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_liftMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure lift motor. Error: " + status.toString());
        }

        m_liftMotorMMV.OverrideBrakeDurNeutral = true;
        m_liftMotor.setRotorPosition(0);
        m_liftMotor.setVoltage(0);
    }

    private void rotateLeftMotorInit(){
        m_rotateMotorLeft = new TalonFX(Constants.Arm.ROTATEMOTORLEFT, "MANIPbus");
        m_rotateMotorLeftMMV = new MotionMagicVoltage(0); 
  
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 50;   

        cfg.Slot0.kP = 2.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

        // tie CANcode on arm rotate shaft to the left motor
        cfg.Feedback.FeedbackRemoteSensorID = m_rotateEncoder.getDeviceID();
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // cfg.Feedback.SensorToMechanismRatio = 1.0;
        // cfg.Feedback.RotorToSensorRatio = 2;

        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_rotateMotorLeft.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate left motor. Error: " + status.toString());
        }

        m_rotateMotorLeftMMV.OverrideBrakeDurNeutral = true;
        m_rotateMotorLeft.setRotorPosition(0);
        m_rotateMotorLeft.setVoltage(0);
    }

    private void rotateRightMotorInit(){
        m_rotateMotorRight = new TalonFX(Constants.Arm.ROTATEMOTORRIGHT, "MANIPbus");
        m_rotateMotorRightMMV = new MotionMagicVoltage(0);  

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 50;   

        cfg.Slot0.kP = 2.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving
    
        // cfg.Feedback.SensorToMechanismRatio = 2F;
        
        // cfg.Voltage.PeakForwardVoltage = 3.2; //3.2V is 20% of 16V
        // cfg.Voltage.PeakForwardVoltage = -3.2; //3.2V is 20% of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_rotateMotorRight.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate right motor. Error: " + status.toString());
        }

        m_rotateMotorRightMMV.OverrideBrakeDurNeutral = true;
        m_rotateMotorRight.setRotorPosition(0);
        m_rotateMotorRight.setVoltage(0);
    }

    // private void liftEncoderInit(){
    //     m_liftEncoder = new CANcoder(Constants.Arm.LIFTENCODER, "MANIPbus");

    //     /* Configure CANcoder */
    //     var cfg = new CANcoderConfiguration();

    //     /* User can change the configs if they want, or leave it empty for factory-default */

    //     m_liftEncoder.getConfigurator().apply(cfg);

    //     /* Speed up signals to an appropriate rate */
    //     m_liftEncoder.getPosition().setUpdateFrequency(100);
    //     m_liftEncoder.getVelocity().setUpdateFrequency(100);
    // }

    private void rotateEncoderInit(){
        m_rotateEncoder = new CANcoder(Constants.Arm.ROTATEENCODER, "MANIPbus");

        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0.4;
        m_rotateEncoder.getConfigurator().apply(cc_cfg);

        /* Speed up signals to an appropriate rate */
        m_rotateEncoder.getPosition().setUpdateFrequency(100);
        m_rotateEncoder.getVelocity().setUpdateFrequency(100);
    }

    @Override
    public void processLoop(double timestamp) {
        
         SystemState newState;
         switch(m_currentState){
             default:
             case NEUTRAL:
                 newState = handleManual();
                 break;
             case GROUND_ANGLE:
                 newState = handleManual();
                 break;
             case HUMAN_FOLD:
                 newState = handleManual();
                 break;
             case MID:
                 newState = handleManual();
                 break;
             case HIGH:
                 newState = handleManual();
                 break;
         }

        if (m_wantedState != m_currentState) {
			m_currentState = newState;
        }
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
 
        // if(!m_liftLimitSwitch.get())
        //     zeroLiftSensor();

        // if(m_rotateLimitSwitch.get())
        //     zeroRotateSensors();

		if(m_intake.getIntakeCurrent()>=200 && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.PLACING && m_intake.getCurrentState() != frc.robot.subsystems.Intake.SystemState.IDLE){
            setWantedState(SystemState.NEUTRAL);
            System.out.println("Intake current threshold hit - Neutral state");
        }
        if(m_controller.getCrossButtonPressed()){
            setWantedState(SystemState.GROUND_ANGLE);
            System.out.println("cross pressed - ground angle state");
        }
        if(m_controller.getCrossButtonReleased()){
            setWantedState(SystemState.NEUTRAL);
            System.out.println("cross released - Neutral state");
        }
        if(m_controller.getCircleButtonPressed()){
            setWantedState(SystemState.MID);
            System.out.println("cross pressed - Mid state");
        }
        if(m_controller.getTriangleButtonPressed()){
            setWantedState(SystemState.NEUTRAL);
            System.out.println("cross pressed - Neutral state");
        }
        if(m_controller.getSquareButtonPressed()){
            setWantedState(SystemState.HIGH);
            System.out.println("cross pressed - High state");
        }
    }
    
    @Override
    public void writePeriodicOutputs(double timestamp)
    {
        switch (m_currentState){
            case GROUND_ANGLE:
				//configRotate(-83000);
                //configExtend(0);

                configRotate(-20.0);
				configLift(0);
                break;
             case MID:
                //configRotate(-40478);
                //configExtend(58652);

				configRotate(-10.0);
                configLift(20.0);
                break;
             case HIGH:
                //configRotate(-40478);
                //configExtend(70000);

			 	configRotate(-10.0);
				configLift(16.0);
                break;
            default:
            case NEUTRAL:
                configRotate(0);
                configLift(0);
                break;

        }
    }

    @Override
    public void stop() {
        
    }

    @Override
    public void outputTelemetry(double timestamp){
        // double calced = m_feedforward.calculate(m_setpoint.position);

        
        // SmartDashboard.putData("Arm State", m_currentState)
        SmartDashboard.putBoolean("LiftLimitSwitch", m_liftLimitSwitch.get());
        SmartDashboard.putBoolean("RotateLimitSwitch", m_rotateLimitSwitch.get());
        SmartDashboard.putString("Lift Pos", m_liftMotor.getPosition().toString());
        SmartDashboard.putString("Rotate Left Pos", m_rotateMotorLeft.getPosition().toString());
        SmartDashboard.putString("Rotate Right Pos", m_rotateMotorLeft.getPosition().toString());
        // SmartDashboard.putString("Lift Encoder Pos", m_liftEncoder.getPosition().toString());
        SmartDashboard.putString("Rotate Encoder Pos", m_rotateEncoder.getPosition().toString());
    }

    private SystemState handleManual(){
        return m_wantedState;
    }

    public void setWantedState(SystemState wanted){
        m_wantedState = wanted;
    }

    public void configLift(double position){
        m_liftMotor.setControl(m_liftMotorMMV.withPosition(position));
        // m_liftMotor.setRotorPosition(position);
    }
   
    public void configRotate(double position){
        m_rotateMotorLeft.setControl(m_rotateMotorLeftMMV.withPosition(position));
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors() {
        // zeroLiftSensor();
        zeroRotateSensors();
		// m_armEncoder.setPosition(0);
		m_rotateEncoder.setPosition(0);
    }
   
    public void zeroLiftSensor(){
        m_liftMotor.setControl(m_liftMotorMMV.withPosition(0));
    }
    public void zeroRotateSensors(){
        m_rotateMotorLeft.setControl(m_rotateMotorLeftMMV.withPosition(0));
    }

    

    @Override
    public String getId() {
        return null;
    }

    // private float ensureRange(float value, float min, float max) {
    //     return Math.min(Math.max(value, min), max);
    //  }
    
}
