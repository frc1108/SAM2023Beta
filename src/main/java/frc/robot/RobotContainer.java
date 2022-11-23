// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.commands.Shoot;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.ShootOnce;
import frc.robot.commands.auto.FourBallShort;
import frc.robot.commands.auto.OneBallAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.WIPRudeTwoBallAuto;
import frc.robot.commands.auto.RudeTwoBallAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  @Log private final DriveSubsystem m_drive = new DriveSubsystem();
  @Log private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log private final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log private final ClimberSubsystem m_climber = new ClimberSubsystem();
  @Log private final LEDSubsystem m_led = new LEDSubsystem();
  private final ColorSubsystem m_color = new ColorSubsystem();
  @Log public final VisionSubsystem m_vision = new VisionSubsystem();

  private final CommandXboxController m_driverCon = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorCon = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
 // private final SendableChooser<Double> delayChooser = new SendableChooser<>();
  private GenericEntry delay;
  private boolean runLight = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystemDefaults();
        // Configure the button bindings
    configureTriggers();
    delay = Shuffleboard.getTab("Live").add("Auto Delay", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties((Map.of("Min", 0, "Max", 10, "Block increment", 1))).getEntry();
    autoChooser.setDefaultOption("Nothing", new WaitCommand(5));
    autoChooser.addOption("2 Ball Auto", new TwoBallAuto(m_drive, m_shooter, m_intake));
    autoChooser.addOption("Rude 2 Ball Auto", new RudeTwoBallAuto(m_drive, m_shooter, m_intake));
    autoChooser.addOption("TEST Rude 2 Ball Auto", new WIPRudeTwoBallAuto(m_drive, m_shooter, m_intake));
    autoChooser.addOption("4 Ball Auto", new FourBallShort(m_drive, m_shooter, m_intake));
    autoChooser.addOption("1 Ball Auto", new OneBallAuto(m_drive, m_shooter, m_intake));
    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void configureSubsystemDefaults() {
    m_drive.setDefaultCommand(run(() -> m_drive.arcadeDrive(
      m_driverCon.getLeftY(),m_driverCon.getRightX()),m_drive)
      .withName("ARCADE DRIVE"));
      
    m_drive.setDefaultSpeed();
      
    m_intake.setDefaultCommand(run(() -> m_intake.intake(
      MathUtil.applyDeadband(m_operatorCon.getLeftY(),
              OIConstants.kOperatorLeftDeadband)),m_intake));
      
    m_climber.setDefaultCommand(run(() -> m_climber.climber(
       MathUtil.applyDeadband(m_operatorCon.getRightY(),
               OIConstants.kOperatorRightDeadband)),m_climber));
  }

  /**   * Use this method to define your trigger-> command mappings. TriggButtons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureTriggers() {
    /** One shot */
    m_operatorCon.povRight().onTrue(m_shooter.shootOnceCommand());

    /** Two shots */
    m_operatorCon.a().onTrue(parallel(new Shoot(m_shooter,0.45),
      print("TWO SHOTS")));

    /** Load in   */
    m_operatorCon.leftBumper().whileTrue(run(()->m_shooter.kick(50), m_shooter));
    /**  */
    m_operatorCon.rightBumper().whileTrue(run(()->m_shooter.kick(-25), m_shooter));
  
    /** Drive faster while holding button */
    m_driverCon.rightBumper().onTrue(runOnce(m_drive::setBurstModeSpeed))
                             .onFalse(runOnce(m_drive::setDefaultSpeed));

    /** Below may or may not be a drift button */
    m_driverCon.rightBumper().onTrue(runOnce(m_drive::setCoast))
                             .onFalse(runOnce(m_drive::setBrake));

    /** Retract intake  */
    m_operatorCon.y().onTrue(m_intake.retractC());    

    /** Extend intake when intaking */
    new Trigger(()->Math.abs(m_operatorCon.getLeftY())>0.2)
        .onTrue(runOnce(m_intake::extend,m_intake));

    /** Toggle climber tilt */
    m_operatorCon.back().onTrue(runOnce(m_climber::toggleTilt,m_climber));

    /** Shooter plate up */
    m_operatorCon.povUp().onTrue(runOnce(m_shooter::plateUp));

    /** Shooter plate down */
    m_operatorCon.povDown().onTrue(runOnce(m_shooter::plateDown).andThen(m_led.chasingHSVCommand(24)));

    /** Toggle shooter tilt */
    m_operatorCon.x().onTrue(runOnce(m_shooter::toggleTilt));

    /** Red LEDs */
    m_driverCon.povUp().onTrue(runOnce(()->m_led.setColor(255, 0, 0),m_led));

    /** Gold LEDs */
    m_driverCon.povRight().onTrue(runOnce(()->m_led.setColor(255, 100, 0),m_led));
    
    /** Blue LEDs */
    m_driverCon.povDown().onTrue(runOnce(()->m_led.setColor(0 , 0, 255),m_led));

    /** Chasing lights LEDs */
    m_driverCon.povLeft().onTrue(m_led.chasingHSVCommand(24));

    // m_driverCon.povUp().onTrue(sequence(runOnce(()->runLight=false),
    //                                     wait(0.1),
    //                                     runOnce(m_led::setRed, m_led)));

    // m_driverCon.povRight().onTrue(sequence(runOnce(()->runLight=false),
    //                                     new WaitCommand(0.1),
    //                                     runOnce(()->m_led.setColor(255,100,0), m_led)));
    
    // m_driverCon.povDown().onTrue(sequence(runOnce(()->runLight=false),
    //                                     new WaitCommand(0.1),
    //                                     runOnce(()->m_led.setColor(0,0,255), m_led)));

    // m_driverCon.povLeft().onTrue(sequence(runOnce(()->runLight=false),
    //                                     (()->m_led.setColor(0,0,255), m_led)));

    // m_driverCon.a().onTrue()

    // new POVButton(m_driverController, 0)
    //     .onTrue(new InstantCommand(()->runLight=false))
    //     .onTrue(new WaitCommand(0.1)
    //     .andThen(new InstantCommand(()->m_led.setRed(),m_led)));
    // new POVButton(m_driverController, 90)
    //     .onTrue(new InstantCommand(()->runLight=false))
    //     .onTrue(new WaitCommand(0.1)
    //     .andThen(new InstantCommand(()->m_led.setColor(255, 100, 0),m_led)));
    // new POVButton(m_driverController, 180)
    //     .onTrue(new InstantCommand(()->runLight=false))
    //     .onTrue(new WaitCommand(0.1)
    //     .andThen(new InstantCommand(()->m_led.setColor(0, 0, 255),m_led)));
    // new POVButton(m_driverController, 270)
    //     .onTrue(new InstantCommand(()->runLight = true))
    //     .onTrue(new RunCommand(()->m_led.chasingHSV(24)).until(()->runLight==false));

    // While driver holds the A button Auto Aim to the High Hub and range to distance    
    // new JoystickButton(m_driverController, XboxController.Button.kA.value)
    //     .whileTrue(new AutoAim(m_drive,m_vision,true,m_driverController));
    m_driverCon.a().whileTrue(new AutoAim(m_drive,m_vision,true,m_driverCon));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new SequentialCommandGroup(new WaitCommand(delay.getDouble(0)), autoChooser.getSelected());
    return sequence(Commands.wait(delay.getDouble(0)), autoChooser.getSelected());
  }

  public void setOperatorRumble() {
    var goodRumble = ((m_color.getBlueFrontMatch()&&DriverStation.getAlliance()==DriverStation.Alliance.Blue))||
                     ((m_color.getRedFrontMatch()&&DriverStation.getAlliance()==DriverStation.Alliance.Red));
    var badRumble = ((m_color.getBlueFrontMatch()&&DriverStation.getAlliance()==DriverStation.Alliance.Red))||
                     ((m_color.getRedFrontMatch()&&DriverStation.getAlliance()==DriverStation.Alliance.Blue));
    m_operatorCon.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, goodRumble ? 0.2 : 0 );
    m_operatorCon.getHID().setRumble(GenericHID.RumbleType.kRightRumble, badRumble ? 1 : 0 );
  }

  public void reset(){
    m_drive.resetOdometry(new Pose2d());
  }
}
