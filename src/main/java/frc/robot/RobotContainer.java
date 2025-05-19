package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
  private final DriveSubsystem SubSys;
  private DefaultDriveCommand DefaultDrive;
  private Joystick joydelicio;

  // The container for the robot contains: subsystems, OI devices, and commands. 
  public RobotContainer() {
    SubSys = new DriveSubsystem();
    joydelicio = new Joystick(Constants.joydelicio_ID);

    DefaultDrive = new DefaultDriveCommand(SubSys, joydelicio);
    SubSys.setDefaultCommand(DefaultDrive);
  }

  // /////////////////////////////////////////////////////////////////////////////////////////////////////
  // /**
  //  * Use this method to define your trigger->command mappings. Triggers can be created via the
  //  * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
  //  * predicate, or via the named factories in {@link
  //  * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
  //  * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
  //  * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
  //  * joysticks}.
  //  */
  // /////////////////////////////////////////////////////////////////////////////////////////////////////
  // private void configureBindings() {
  //   // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
  //   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  // }
  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  * @return the command to run in autonomous
  //  */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  // }
}
