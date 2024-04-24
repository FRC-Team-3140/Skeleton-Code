// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.SwerveController;
import frc.robot.Libs.ControllerHelper;
import frc.robot.Subsystems.SwerveDrive;

public class RobotContainer implements Constants{
  public static SwerveDrive swerve = SwerveDrive.getInstance();
  public static ControllerHelper controller = new ControllerHelper(0);
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveController(swerve, maxChassisSpeed, maxChassisTurnSpeed));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
