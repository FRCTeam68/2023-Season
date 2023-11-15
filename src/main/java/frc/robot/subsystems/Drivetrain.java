
package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.CTRSwerve.CTRSwerveDrivetrain;
import frc.robot.CTRSwerve.SwerveDriveConstantsCreator;
import frc.robot.CTRSwerve.SwerveDriveTrainConstants;
import frc.robot.CTRSwerve.SwerveModuleConstants;


public class Drivetrain implements Subsystem {

    /*
     * DRIVE CONTROLLER SCHEME (PLEASE KEEP UPDATED)
     * 
     * MAPPED BUTTONS:
     * A -> Auto Balance
     * B -> Cruise
     * X -> Limelight Cruise
     * 
     * RBumper -> Lock to 180 degrees
     * LBumper -> Lock to 360 degrees
     * 
     * LAxis -> Drive
     * RAxis -> Rotate
     */

    // Actually have no clue what these do. Have too much of a dog brain for this
    public static final double[] XY_Axis_inputBreakpoints = { -1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1 };
    public static final double[] XY_Axis_outputTable = { -1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0 };
    public static final double[] RotAxis_inputBreakpoints = { -1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1 };
    public static final double[] RotAxis_outputTable = { -1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0 };

    private final SlewRateLimiter slewRot = new SlewRateLimiter(1880);

    private enum SystemState {
        IDLE,
        MANUAL_CONTROL,
        TRAJECTORY_FOLLOWING, // X,Y axis speeds relative to field
        AUTO_BALANCE
    }

    public enum WantedState {
        IDLE,
        MANUAL_CONTROL,
        TRAJECTORY_FOLLOWING,
        AUTO_BALANCE
    }

    private static class PeriodicIO {
        double modifiedJoystickX;
        double modifiedJoystickY;
        double modifiedJoystickR;
    }

    private final PeriodicIO periodicIO = new PeriodicIO();

    private SystemState currentState = SystemState.MANUAL_CONTROL;
    private WantedState wantedState = WantedState.MANUAL_CONTROL;

    private final XboxController controller;

    private double[] autoDriveSpeeds = new double[2];
    public final static int Num_Modules = 4;

    private final CTRSwerveDrivetrain drivetrain;

    public SwerveModuleState[] trajectoryStates = new SwerveModuleState[4];

    private boolean balancedX = false;

    private double pitchAngle = 0;

    public Drivetrain(XboxController controller) {
        // yawCtrl.enableContinuousInput(-Math.PI, Math.PI); //TODO check if Pigeon
        // output rolls over
        
        Slot0Configs steerGains = new Slot0Configs();
        Slot0Configs driveGains = new Slot0Configs();
    
        {
            steerGains.kP = 30;
            steerGains.kD = 0.2;
            driveGains.kP = 2;
        }

        SwerveDriveConstantsCreator constants =  new SwerveDriveConstantsCreator(Constants.Swerve.gearRatio, Constants.Swerve.angleGearRatio, 
        2, 400, steerGains, driveGains, false);
    

        drivetrain = new CTRSwerveDrivetrain(new SwerveDriveTrainConstants().withTurnKp(Constants.Swerve.angleKP).withPigeon2Id(50).withCANbusName("MANIPbus"),

        constants.createModuleConstants(Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.driveMotorID, 
                Constants.Swerve.Mod0.canCoderID, Constants.Swerve.Mod0.dobOffset, 0.260, 0.222),
                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod1.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod1.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod1.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod1.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationX(0.260)
                        .withLocationY(-0.222),

                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod2.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod2.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod2.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod2.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationX(-0.260)
                        .withLocationY(0.222),

                new SwerveModuleConstants().withCANcoderId(Constants.Swerve.Mod3.canCoderID)
                        .withDriveMotorId(Constants.Swerve.Mod3.driveMotorID)
                        .withSteerMotorId(Constants.Swerve.Mod3.angleMotorID)
                        .withCANcoderOffset(Constants.Swerve.Mod3.dobOffset)
                        .withDriveMotorGearRatio(Constants.Swerve.gearRatio)
                        .withSteerMotorGearRatio(Constants.Swerve.angleGearRatio)
                        .withWheelRadius(2)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSlipCurrent(400)
                        .withLocationY(-0.260)
                        .withLocationX(-0.222) );

        drivetrain.seedFieldRelative();

        this.controller = controller;
    }

    @Override
    public void processLoop(double timestamp) {
        SystemState newState;
        switch (currentState) {
            default:
            case MANUAL_CONTROL:
                newState = handleManualControl();
                break;
            case AUTO_BALANCE:
                newState = handleManualControl();
                break;
            case TRAJECTORY_FOLLOWING:
                newState = handleTrajectoryFollowing();
                break;
            case IDLE:
                newState = handleManualControl();
                break;

        }
        if (newState != currentState) {
            currentState = newState;
        }

    }

    @Override
    public void readPeriodicInputs(double timestamp) {

        periodicIO.modifiedJoystickX = controller.getLeftX();
        periodicIO.modifiedJoystickY = controller.getLeftY();

        periodicIO.modifiedJoystickR = slewRot
                .calculate(controller.getRightX());

        checkButtons();

    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        ChassisSpeeds chassis = new ChassisSpeeds(0,0,0);


        switch (currentState) {
            case TRAJECTORY_FOLLOWING:
                SwerveDriveKinematics.desaturateWheelSpeeds(trajectoryStates, Constants.Swerve.maxSpeed);
                chassis = drivetrain.getKinematics().toChassisSpeeds(trajectoryStates[0], trajectoryStates[1], trajectoryStates[2], trajectoryStates[3]
                );
                break;
            case AUTO_BALANCE:
                autoBalance();
                break;
            case MANUAL_CONTROL:
                chassis = new ChassisSpeeds(periodicIO.modifiedJoystickY, periodicIO.modifiedJoystickX,
                        periodicIO.modifiedJoystickR);
                break;
            default:
            case IDLE:
                break;

        }

        SmartDashboard.putNumber("Drivetrain/Chassis X", chassis.vxMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Chassis Y", chassis.vyMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Chassis Angle", chassis.omegaRadiansPerSecond);

        drivetrain.driveFieldCentric(chassis);
        // updateStateVariables(moduleStates);
    }

    private SystemState defaultStateChange() {
        switch (wantedState) {
            /*
             * case IDLE:
             * return SystemState.IDLE;
             */
            case AUTO_BALANCE:
                return SystemState.AUTO_BALANCE;
            case TRAJECTORY_FOLLOWING:
                return SystemState.TRAJECTORY_FOLLOWING;
            default:
            case MANUAL_CONTROL:
                return SystemState.MANUAL_CONTROL;
        }
    }

    @Override
    public void periodic() {

    }

    private void checkButtons() {
        if (controller.getAButtonPressed())
            setWantedState(WantedState.AUTO_BALANCE);
        if (controller.getAButtonReleased())
            setWantedState(WantedState.MANUAL_CONTROL);

        pitchAngle = drivetrain.getPitch() - Constants.BALANCED_OFFESET;

        if (Math.abs(pitchAngle) >= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES) && balancedX) {
            balancedX = false;
        } else if (!balancedX && Math.abs(pitchAngle) <= Math.abs(Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES)) {
            balancedX = true;
        }


    }

    private void autoBalance() {
        // TODO: Cap the angles given so we never calculate above a certain value.
        double xAxisRate = 0;

        if (!balancedX && pitchAngle > 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate = Math.min(4, Math.abs(Math.sin(pitchAngleRadians)));
        }

        if (!balancedX && pitchAngle < 0) {

            double pitchAngleRadians = pitchAngle * (Math.PI / 180.0);
            xAxisRate = Math.min(4, Math.abs(Math.sin(pitchAngleRadians)) * -0.3);
        }

        drive(xAxisRate * Constants.BALANCEDMAXSPEED, 0, 0.0, true);

    }

    @Override
    public void zeroSensors() {
        drivetrain.resetOdometry();
        drivetrain.driveFullyFieldCentric(0, 0, getYaw());
    }

    @Override
    public void stop() {
        drivetrain.driveFullyFieldCentric(0, 0, getYaw());
    }

    public boolean getLeftTrigger() {
        return (controller.getRawAxis(2) == 1) ? true : false;
    }

    public boolean getRightTrigger() {
        return (controller.getRawAxis(3) == 1) ? true : false;
    }

    @Override
    public void outputTelemetry(double timestamp) {
        SmartDashboard.putNumber("Yaw", getYaw().getRadians());
        SmartDashboard.putNumber("Pitch", pitchAngle);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void initAutonPosition(PathPlannerTrajectory.PathPlannerState state) {
        // ErrorCode errorCode = pigeon.setYaw(state.holonomicRotation.getDegrees(),
        // 100);
        drivetrain.getOdometry().resetPosition(getYaw(), getModulePositions(),
                new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
    }

    public void zeroGyroscope() {
        drivetrain.seedFieldRelative();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        ChassisSpeeds chassis = drivetrain.getKinematics().toChassisSpeeds(desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
        );

        drivetrain.driveFieldCentric(chassis);
    }

    public void setModuleStatesFromTrajectory(SwerveModuleState[] states) {
        trajectoryStates = states;
    }

    private SystemState handleManualControl() {

        return defaultStateChange();
    }

    private SystemState handleTrajectoryFollowing() {
        return defaultStateChange();
    }

    public Pose2d getPose() {
        return drivetrain.getPoseMeters();
    }

    public void setAutoDriveSpeeds(double xSpeed, double ySpeed) {
        autoDriveSpeeds[0] = xSpeed;
        autoDriveSpeeds[1] = ySpeed;
    }

    public CTRSwerveDrivetrain getBaseDrivetrain(){
        return drivetrain;
    }

    public SwerveModulePosition[] getModulePositions() {

        return drivetrain.getSwervePositions();
    }


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        drivetrain.driveFieldCentric(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    public Rotation2d getYaw() {

        return Rotation2d.fromDegrees(drivetrain.getYaw());
    }

    @Override
    public String getId() {
        return "Drivetrain";
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}