package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;


public class Robot extends TimedRobot {

  private AutoDriveCommand autodrive;
  private DriveCommand DefaultDrive;
  private DriveSubsystem SubSys;
  private RobotContainer RobotContainer;
  private Joystick joydelicio;



@Override
public void robotInit() {
    SubSys = new DriveSubsystem();
    RobotContainer = new RobotContainer();
    joydelicio = new Joystick(Constants.joydelicio_ID);
    DefaultDrive = new DriveCommand(SubSys, joydelicio);
}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autodrive = new AutoDriveCommand(SubSys, 0.2);

    if (autodrive != null) {
      autodrive.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

}

  @Override
  public void teleopInit() {
    if (autodrive != null) {
      autodrive.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
