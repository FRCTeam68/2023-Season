// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auton.Autons;
import frc.robot.subsystems.Intake;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private Command autonomousCommand;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static CTREConfigs ctreConfigs;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.startSubsystemThreads();
    Autons.checkPath();
  }

  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    m_robotContainer.enableState = RobotContainer.EnableState.AUTON;
    
    autonomousCommand = Autons.test(m_robotContainer.drivetrain);

    
    if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
    
  }

  @Override
  public void autonomousPeriodic() {
   /* 
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
        break;
    }
    */
  }

  @Override
  public void teleopInit() {
    m_robotContainer.enableState = RobotContainer.EnableState.TELEOP;
  
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
