// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDrive extends Command {
  /** Creates a new ArcadeDrive. */
  private Drivetrain m_drivetrain;
  private Joystick m_joystick;

  private double kSpeedConstant = 1;
  private double kTurnConstant = 1;

  private double driveSpeed;
  private double turnSpeed;
  
  public ArcadeDrive(Drivetrain drivetrain, Joystick joystick) {
    m_drivetrain = drivetrain;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setLeftSpeed(0);
    m_drivetrain.setRightSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_drivetrain.getLimitSwitch()) { // if limit switch isn't hit, keep driving
      SmartDashboard.putString("Limit Switch Status", "Limit Switch is fine");
      driveSpeed = kSpeedConstant*m_joystick.getRawAxis(1);
      turnSpeed = kTurnConstant*m_joystick.getRawAxis(0);

      m_drivetrain.setLeftSpeed(Math.max(driveSpeed - turnSpeed, 1));
      m_drivetrain.setRightSpeed(Math.max(driveSpeed + turnSpeed, 1));
    } else {
      // something something put a value to smartdashboard
      SmartDashboard.putString("Limit Switch Status", "Limit Switch is hit!!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
