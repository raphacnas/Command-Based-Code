package frc.robot;

import frc.robot.commands.TagFollowerCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSimSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  private final DriveSubsystem DriveSubSys = new DriveSubsystem();
  private final VisionSimSubsystem SimSubSys = new VisionSimSubsystem();
  private Joystick joydelicio = new Joystick(Constants.joydelicio_ID);

  public RobotContainer() {
    DriveCommand DefaultDrive = new DriveCommand(DriveSubSys, joydelicio);
    DriveSubSys.setDefaultCommand(DefaultDrive);
  }

  public void simulationPeriodic() {
    SimSubSys.updateSimPose(DriveSubSys.getPose2D());
}

  // private void configureBindings() {}
  
  public Command getAutonomousCommand() {
    return new TagFollowerCommand(DriveSubSys, SimSubSys);
  }
}
