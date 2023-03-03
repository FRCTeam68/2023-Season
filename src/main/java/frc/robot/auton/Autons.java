package frc.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.commands.*;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Autons {
    private static final PathPlannerTrajectory straightLine = PathPlanner.loadPath("striahgtlinetest", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);


public static Command test(Drivetrain driveTrain){
    final PathPlannerCommand straightLineTest = new PathPlannerCommand(straightLine, driveTrain, true);

    return new SequentialCommandGroup(
    new InstantCommand(() -> driveTrain.drive(0, 0, 0, true)),
    new WaitCommand(3),
    straightLineTest 
    );
}

}