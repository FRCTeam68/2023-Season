package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.TalonFX;
// import com.ctre.phoenixpro.StatusSignalValue;
// import com.ctre.phoenixpro.controls.NeutralOut;

//1a
import com.ctre.phoenixpro.controls.VelocityVoltage;

// import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.controls.VoltageOut;

// import edu.wpi.first.wpilibj.XboxController;
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
        PLACING
    }

    public enum WantedState{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        PLACING
    }

    private SystemState m_currentState = SystemState.IDLE;
    private WantedState m_wantedState = WantedState.IDLE;

    private boolean m_haveCube;
    private boolean m_haveCone;

    private double m_currentStateStartTime = 0;

    protected TalonFX m_intakeMotor;

    //0
    protected VoltageOut m_voltageOut;

    //1b,2b,3b
    protected VelocityVoltage m_voltageVelocity;


    private final PS4Controller m_controller;


    public Intake(PS4Controller controller){
        m_controller = controller;

        intakeMotorInit();

        m_haveCone = false;
        m_haveCube = false;
    }

    //0, 13
    private void intakeMotorInit(){
        m_intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        //1c
        cfg.Slot0.kP = 0.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        // cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving
   
        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        // cfg.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        // cfg.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        // cfg.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        // cfg.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts

        //1d
        cfg.Voltage.PeakForwardVoltage = 2;
        cfg.Voltage.PeakReverseVoltage = -2;

        // /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        // cfg.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        // cfg.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        // cfg.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

        //1e
        // Peak output of 40 amps
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -40;


        // any unmodified configs in a configuration object are *automatically* factory-defaulted;
        // user can perform a full factory default by passing a new configuration object
        // m_intakeMotor.getConfigurator().apply(new TalonFXConfiguration());

        // cfg.Voltage.PeakForwardVoltage = 2;  // 2V of 16V
        // cfg.Voltage.PeakForwardVoltage = -2; //-2V of 16V

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_intakeMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure lift motor. Error: " + status.toString());
        }

        // liftMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.setRotorPosition(0);

        /* Start at velocity 0, enable FOC, no feed forward, use slot 0, NeutralBrake ON when 0 volts */
        //    m_voltageVelocity = new VelocityVoltage(0, true, 0, 0, true);

        // m_voltageVelocity = new VelocityVoltage(0).withSlot(0);

        m_voltageOut = new VoltageOut(0);

        setIntakeSpeed(0);
        // m_intakeMotor.setVoltage(0);
    }

    private void intakeMotorInit2(){
        m_intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = 1.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        // cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

        cfg.Voltage.PeakForwardVoltage = 8;
        cfg.Voltage.PeakReverseVoltage = -8;
        
          // Peak output of 40 amps
        cfg.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        cfg.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_intakeMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure lift motor. Error: " + status.toString());
        }

        // liftMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.setRotorPosition(0);

        // m_voltageVelocity = new VelocityVoltage(0).withSlot(0);
        m_voltageVelocity = new VelocityVoltage(0, false, 0, 0, false);

        setIntakeSpeed(0);
    }

    private void intakeMotorInit3(){
        m_intakeMotor = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "MANIPbus");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        //11
        // cfg.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        // cfg.Slot0.kI = 0.5; // An error of 1 rps increases output by 0.5 V each second
        // cfg.Slot0.kD = 0.001; // An acceleration of 1 rps/s results in 0.001 V output
        // cfg.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // cfg.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction

        // //10
        cfg.Slot0.kP = 0.1; //12 0.05;  //11 0.1;  //10 0.1; //9 0;  //8 0.4;  //7 0.7; //6 0.3;  //5 .1; 
        cfg.Slot0.kI = 0;               //11 0.001;                  //8 0.05;          //6 0.1;  //5 .05; 
        cfg.Slot0.kD = 0;               //11 1;
        cfg.Slot0.kV = 0; 
        cfg.Slot0.kS = 0;

        // cfg.Voltage.PeakForwardVoltage = 8;
        // cfg.Voltage.PeakReverseVoltage = -8;
        
          // Peak output of 40 amps
        // cfg.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        // cfg.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit = 15.0;
        // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_intakeMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure lift motor. Error: " + status.toString());
        }

        // liftMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.setRotorPosition(0);

        // m_voltageVelocity = new VelocityVoltage(0).withSlot(0);
        m_voltageVelocity = new VelocityVoltage(0, false, 0, 0, false);

        setIntakeSpeed(0);
    }

    @Override
    public void processLoop(double timestamp) {
        // TODO Auto-generated method stub
        SystemState newState;
        switch (m_currentState){
            default:
            case INTAKING_CONE:
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
        if (newState != m_currentState) {
			m_currentState = newState;
			m_currentStateStartTime = timestamp;
		}
    }

    @Override
    public void readPeriodicInputs(double timestamp){
        if (m_controller.getL1ButtonPressed()){
            m_wantedState =  WantedState.INTAKING_CONE;
            System.out.println("L1 pressed - intake - intaking CONE state");
        }
        if (m_controller.getL1ButtonReleased()){
            m_wantedState = WantedState.IDLE;
            System.out.println("L1 released - intake - idle state");
        }
        if (m_controller.getL2ButtonPressed()){
            m_wantedState = WantedState.INTAKING_CUBE;
            System.out.println("L2 pressed - intake - intaking CUBE state");
        }  
        if (m_controller.getL2ButtonReleased()){
            m_wantedState = WantedState.IDLE;
            System.out.println("L2 released - intake - idle state");
        }  
        if (m_currentState == SystemState.INTAKING_CONE && getIntakeCurrent() > 200){
            System.out.println("intake cone - stall current 200 reached - have CONE");
            new SequentialCommandGroup(
                new WaitCommand(5),
                new InstantCommand(()-> setWantedState(WantedState.IDLE))
            );
            m_haveCone = true;
        }
        if (m_currentState == SystemState.INTAKING_CUBE && getIntakeCurrent() > 100){
            System.out.println("intake cube - stall current 100 reached - have CUBE");
            new SequentialCommandGroup(
                new WaitCommand(5),
                new InstantCommand(()-> setWantedState(WantedState.IDLE_CUBE))
            );
            m_haveCube = true;
        }

        if(m_haveCone){
            m_wantedState = WantedState.IDLE;
            if (m_controller.getL1ButtonPressed()){
                m_wantedState =  WantedState.INTAKING_CONE;
                System.out.println("have CONE - L1 pressed - intaking CONE again");
            }
        }
        if (m_haveCube){
            m_wantedState = WantedState.IDLE_CUBE;
            if (m_controller.getL2ButtonPressed()){
                m_wantedState =  WantedState.INTAKING_CONE;
                System.out.println("have CUBE - L2 pressed - intaking CUBE again");
            }
        }

            
        if (m_controller.getR2ButtonPressed()){
            m_wantedState = WantedState.PLACING;
            m_haveCone = false;
            m_haveCube = false;
            System.out.println("R2 Pressed - intake - placing state");
        }
            
        if (m_controller.getR2ButtonReleased()){
            m_wantedState = WantedState.IDLE;
            System.out.println("R2 Released - intake - idle state");
        }

    }

    @Override
    public void writePeriodicOutputs(double timestamp){
        switch(m_currentState){

            case INTAKING_CONE:
                setIntakeSpeed(-.7);         //orig, 13
                // setIntakeSpeed(-4);          //1,2
                // setIntakeSpeed( 10);      //0
                // setIntakeSpeed( 40);      //3-12
                break;
            case INTAKING_CUBE:
                setIntakeSpeed(-.3);         //orig, 13
                // setIntakeSpeed(-2);          //1,2
                // setIntakeSpeed(5);       //0
                // setIntakeSpeed( 20);      //3-12
                break;
            case PLACING:
                setIntakeSpeed(.5);          //orig, 13
                // setIntakeSpeed(1);           //1,2
                // setIntakeSpeed(2);       //0
                // setIntakeSpeed( -10);           //3-12
                break;
            case IDLE_CUBE:
                setIntakeSpeed(-.1);         //orig, 13
                // setIntakeSpeed(-5);            //1,2
                break;
            default:
            case IDLE:
                setIntakeSpeed(0);
                break;
        }
    }


    private SystemState handleManual(){
        switch (m_wantedState){
            case INTAKING_CONE:
                return SystemState.INTAKING_CONE;
            case INTAKING_CUBE:
                return SystemState.INTAKING_CUBE;
            case PLACING:
                return SystemState.PLACING;
            case IDLE_CUBE:
                return SystemState.IDLE_CUBE;
            default:
            case IDLE:
                return SystemState.IDLE;
		}
    }

    //13
    public void setIntakeSpeed(double percent){
        var percentOutput = percent * 12;
        m_intakeMotor.setControl(m_voltageOut.withOutput(percentOutput));
    }

    //0
    public void setIntakeSpeed0(double speed){
        // m_intakeMotor.set(ControlMode.PercentOutput, speed);
        // m_intakeMotor.setControl(m_voltageVelocity.withVelocity(speed));

        // main robot code, command 12 V output
        // m_intakeMotor.setControl(m_voltageVelocity.withOutput(speed);

        m_intakeMotor.setControl(m_voltageOut.withOutput(speed));
        
        // m_intakeMotor.setVoltage(speed);
    }

    public void setIntakeSpeed1(double speed){
        m_intakeMotor.setControl(m_voltageOut.withOutput(speed));
    }

    //2,3
    public void setIntakeSpeed3(double speed){
        // System.out.println("speed" + speed);
        
        m_intakeMotor.setControl(m_voltageVelocity.withVelocity(speed));
        //12 m_intakeMotor.setControl(m_voltageVelocity.withVelocity(speed).withFeedForward(10));
    }

    
    public double getIntakeCurrent(){
        return m_intakeMotor.getStatorCurrent().getValue();
    }
    public double getSupplyCurrent(){
        return m_intakeMotor.getSupplyCurrent().getValue();
    }
    @Override
    public void outputTelemetry(double timestamp){
        SmartDashboard.putNumber("Stator Current", getIntakeCurrent());
        SmartDashboard.putNumber("Supply Current", getSupplyCurrent());
        SmartDashboard.putString("intake volts", m_intakeMotor.getSupplyVoltage().toString());
        SmartDashboard.putString("intake Pos", m_intakeMotor.getPosition().toString());
        SmartDashboard.putString("intake Vel", m_intakeMotor.getVelocity().toString());
        SmartDashboard.putBoolean("HaveCube", m_haveCube);
        SmartDashboard.putBoolean("HaveCone", m_haveCone);
    }
    @Override
    public void stop() {
        // TODO Auto-generated method stub
        setIntakeSpeed(0);
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
		m_wantedState = wantedState;
	}

    public SystemState getCurrentState(){
        return m_currentState;
    }

}