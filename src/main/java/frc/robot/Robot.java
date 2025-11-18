package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

  private Command autodrive;
  private RobotContainer RobotContainer;

  @Override
  public void robotInit() {
      RobotContainer = new RobotContainer();
      
      Logger.recordMetadata("ProjectName", "Binga");
      Logger.recordMetadata("RuntimeType", RobotBase.getRuntimeType().toString());
  
      if (RobotBase.isSimulation()) {
          Logger.addDataReceiver(new WPILOGWriter("logs")); // Caminho relativo
          Logger.addDataReceiver(new NT4Publisher());
      } else {
          Logger.addDataReceiver(new WPILOGWriter("logs")); // Caminho relativo
          Logger.addDataReceiver(new NT4Publisher());
      }
  
      try {
          Logger.start();
      } catch (Exception e) {
          e.printStackTrace();
          System.out.println("Failed to start AdvantageKit Logger. Check directory permissions or configuration.");
      }
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
    autodrive = RobotContainer.getAutonomousCommand();

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
