/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.auto.groups.AutoTest2;
import frc.robot.controllers.AxisJoystickButton;
import frc.robot.controllers.AxisJoystickButton.ThresholdType;
import frc.robot.subsystems.*;
import frc.robot.subsystems.components.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // components
  public final XboxController driver = new XboxController(Constants.OIConstants.xboxControllerDriver);
  private final XboxController assist = new XboxController(Constants.OIConstants.xboxControllerAssist);

  //private final ColorSensor colorSensor = new ColorSensor();
  private final RangeSensors rangeSensors = new RangeSensors();
  private final LimeLight limeLight = new LimeLight();
  private final NavX navX = new NavX();

  public final DriveTrain driveTrain = new DriveTrain(navX);
  private final Lift elevator = new Lift();
  private final Indexer indexer = new Indexer(rangeSensors);
  private final IntakeElbow intakeElbow = new IntakeElbow();
  private final IntakeWheels intakeWheels = new IntakeWheels(assist);
  private final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  private final Bombardier bombardier = new Bombardier(indexer, turret, shooter, limeLight, intakeWheels);

  private SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(chooser);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveTrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
                                                          //FWD                   ROT
        new RunCommand(() -> driveTrain.arcadeDrive(-driver.getRightY(),driver.getLeftX()) , driveTrain));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton buttonA_dr = new JoystickButton(driver, Constants.OIConstants.buttonA);
    JoystickButton buttonY_dr = new JoystickButton(driver, Constants.OIConstants.buttonY);
    JoystickButton buttonSelect_dr = new JoystickButton(driver, Constants.OIConstants.buttonSelect);
    AxisJoystickButton triggerRight = new AxisJoystickButton(driver, Constants.OIConstants.rightTrigger, 0.5, ThresholdType.GREATER_THAN);
    
    triggerRight.whenPressed(new InstantCommand(bombardier::targetNoParams,bombardier));
    triggerRight.whenReleased(new InstantCommand(bombardier::stopTargetNoParams,bombardier));

    buttonA_dr.whenPressed(new InstantCommand(elevator::up,elevator));
    buttonY_dr.whenPressed(new InstantCommand(elevator::down, elevator));

    buttonSelect_dr.whenPressed(new InstantCommand(bombardier::togglFailSafe,bombardier));

    JoystickButton buttonA_as = new JoystickButton(assist, Constants.OIConstants.buttonA);
    JoystickButton buttonY_as = new JoystickButton(assist, Constants.OIConstants.buttonY);
    JoystickButton buttonX_as = new JoystickButton(assist, Constants.OIConstants.buttonX);
    JoystickButton buttonB_as = new JoystickButton(assist, Constants.OIConstants.buttonB);

    JoystickButton buttonBumperRight_as = new JoystickButton(assist, Constants.OIConstants.buttonBumperRight);
    JoystickButton buttonBumperLeft_as = new JoystickButton(assist, Constants.OIConstants.buttonBumperLeft);

    buttonX_as.whenPressed(new InstantCommand(intakeWheels::on,intakeWheels));
    buttonB_as.whenPressed(new InstantCommand(intakeWheels::off,intakeWheels));

    buttonA_as.whenPressed(new InstantCommand(intakeElbow::down,intakeElbow));
    buttonY_as.whenPressed(new InstantCommand(intakeElbow::home,intakeElbow));

    buttonBumperRight_as.whenPressed(new InstantCommand(intakeWheels::reverse,intakeWheels));
    buttonBumperRight_as.whenReleased(new InstantCommand(intakeWheels::returnToPrevState,intakeWheels));

    buttonBumperLeft_as.whenPressed(new InstantCommand(indexer::reverseIndexer, indexer));
    buttonBumperLeft_as.whenReleased(new InstantCommand(indexer::off, indexer));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory1, Trajectory trajectory2) {
    return new AutoTest2(driveTrain, intakeElbow, intakeWheels, indexer, trajectory1, trajectory2);
  }

  public void disabledLEDS() {
    limeLight.ledOff();
  }

  public void enableAutoIndexing() {
    indexer.setAutoIndexOn();
  }
}
