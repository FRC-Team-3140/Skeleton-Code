// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SwerveController;
import frc.robot.Libs.ControllerHelper;
import frc.robot.Subsystems.SwerveDrive;

public class RobotContainer implements Constants{
  public static SwerveDrive swerve = SwerveDrive.getInstance();
  public static ControllerHelper controller = new ControllerHelper(0);
  public static XboxController controller2 = new XboxController(1);
  public static SendableChooser<Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveController(swerve, maxChassisSpeed, maxChassisTurnSpeed));
    configureBindings();
    autoChooser.addOption("Auto1",  AutoBuilder.buildAuto("Example Auto"));
    SmartDashboard.putData("Autos", autoChooser);
    
  }

  private void configureBindings() {
    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand(()-> {swerve.resetGyro();}));

    // new JoystickButton(controller2, Button.kY.value).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
    // new JoystickButton(controller2, Button.kB.value).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

    // new JoystickButton(controller2, Button.kA.value).whileTrue(swerve.sysIdDynamic(Direction.kForward));
    // new JoystickButton(controller2, Button.kX.value).whileTrue(swerve.sysIdDynamic(Direction.kReverse));



    

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
