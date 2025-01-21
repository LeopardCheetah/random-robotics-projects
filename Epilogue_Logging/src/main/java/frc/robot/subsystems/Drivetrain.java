// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private TalonFX m_leftFrontMotor = new TalonFX(DrivetrainConstants.kleftFrontMotorID);
  private TalonFX m_leftBackMotor = new TalonFX(DrivetrainConstants.kLeftBackMotorID);
  private TalonFX m_rightFrontMotor = new TalonFX(DrivetrainConstants.kRightFrontMotorID);
  private TalonFX m_rightBackMotor = new TalonFX(DrivetrainConstants.kRightBackMotorID);
  // so set inverted is gone


  // add limit switch
  private DigitalInput m_limitSwitch = new DigitalInput(DIOConstants.kDrivetrainLimitSwitchID); 

  public Drivetrain() {}

  public void setLeftSpeed(double speed){
    m_leftFrontMotor.set(speed);
    m_leftBackMotor.set(speed);
  }

  public void setRightSpeed(double speed){
    // notice the negative sign -- this is meant to mimic setting inverse speeds
    m_rightFrontMotor.set(-speed);
    m_rightBackMotor.set(-speed);
  }

  public boolean getLimitSwitch(){
    return m_limitSwitch.get();
  } 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
