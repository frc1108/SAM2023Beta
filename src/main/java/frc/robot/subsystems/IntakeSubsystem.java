// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
  private final DoubleSolenoid m_intakeExtender = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, IntakeConstants.kIntakeExtendChannel, IntakeConstants.kIntakeRetractChannel);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    stopIntake();
    m_intakeExtender.set(Value.kReverse);
    m_intake.restoreFactoryDefaults();
    m_intake.setIdleMode(IdleMode.kBrake);
    m_intake.setInverted(false);
    m_intake.setSmartCurrentLimit(40, 60);
    m_intake.burnFlash();
  }
 

  public Command toggleCommand() {
    return runOnce(m_intakeExtender::toggle,this);
  }
  public void extend() {
    m_intakeExtender.set(Value.kForward);
  }

  /** Return Command to retract intake */
  public Command retractC() {
    return runOnce(()->m_intakeExtender.set(Value.kReverse))
           .alongWith(print("RETRACT INTAKE")).withName("retractC");
  }

  /** Return Command to deploy intake */
  public Command deployC() {
    return runOnce(()->m_intakeExtender.set(Value.kForward))
           .alongWith(print("DEPLOY INTAKE")).withName("deployC");
  }
  
  /** Stop intake motor */
  public void stopIntake() {
    m_intake.stopMotor();
  }

  /** Set the intake voltage */
  public void intake() {
    m_intake.setVoltage(IntakeConstants.kIntakeSpeed);
  }
  
  /** Set the intake voltage with modifier
   * @param modifier multiplies intake speed between 0-1
   */
  public void intake(double modifier) {
    m_intake.setVoltage(IntakeConstants.kIntakeSpeed*MathUtil.clamp(modifier,-1,1));
  }
  
  /** Returns Trigger if intake is running */ 
  public Trigger isIntaking() {
    return new Trigger(()->(m_intake.get() > 0.2));
  }

  public Trigger isStopped() {
    return new Trigger(()->((Math.abs(m_intake.get()) < 0.2))).debounce(0.2);
  }
}
