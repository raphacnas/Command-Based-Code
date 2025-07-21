package frc.robot;

import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final DriveSubsystem SubSys = new DriveSubsystem();
  private Joystick joydelicio = new Joystick(Constants.joydelicio_ID);

  public RobotContainer() {
    DriveCommand DefaultDrive = new DriveCommand(SubSys, joydelicio);
    SubSys.setDefaultCommand(DefaultDrive);
  }

  // private void configureBindings() {}
  
  public Command getAutonomousCommand() {
    return new AutoDriveCommand(SubSys, 0.5);
  }
}
