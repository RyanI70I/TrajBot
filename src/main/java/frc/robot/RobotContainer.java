/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.DriveManual;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.sensorTest;
import frc.robot.commands.setPoseHome;
import frc.robot.helpers.trajectoryhelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   XboxController controller = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Drivetrain drivetrain = new Drivetrain();
  //private final DriveManual driveManual = new DriveManual(controller.getBumper(GenericHID.Hand.kLeft), controller.getTriggerAxis(GenericHID.Hand.kRight), controller.getTriggerAxis(GenericHID.Hand.kLeft), controller.getX(Hand.kLeft), controller.getTriggerAxis(GenericHID.Hand.kRight), drivetrain);




  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drivetrain.initDrive();
    // Configure the button bindings

    drivetrain.setDefaultCommand(new DriveManual(() -> blb(), () -> lTrigger(), () -> rTrigger(), () -> getLeftX(), () -> getRightX(), drivetrain));

    configureButtonBindings();

  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton test = new JoystickButton(controller, XboxController.Button.kX.value);
    final JoystickButton test2 = new JoystickButton(controller, XboxController.Button.kY.value);
    final JoystickButton test3 = new JoystickButton(controller, XboxController.Button.kB.value);
    test2.whenHeld(new sensorTest(drivetrain));
    test3.whenHeld(new setPoseHome(drivetrain));


      test.toggleWhenPressed(new RamseteCommand(trajectoryhelper.trajectory(), drivetrain::getPose, new RamseteController(2, .7), new DifferentialDriveKinematics(.62), drivetrain::setVelocity, drivetrain));

  }
  public double rTrigger(){
    return controller.getTriggerAxis(Hand.kRight);
  }
  public double lTrigger(){
    return controller.getTriggerAxis(Hand.kLeft);
  }
  public boolean blb(){
    return controller.getBumper(GenericHID.Hand.kLeft);
  }
  public double getLeftX(){
    return controller.getX(Hand.kLeft);
  }
  public double getRightX(){
    return controller.getX(Hand.kRight);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
