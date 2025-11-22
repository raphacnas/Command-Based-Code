package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.GoToPose;
import frc.robot.commands.TagFollower;
import frc.robot.simulator.simCommands.TagFollowerSim;
import frc.robot.simulator.simSubsystems.SimSubsystem;
import frc.robot.simulator.simSubsystems.VisionSimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  private final DriveSubsystem SubSys = new DriveSubsystem();
  private final Joystick joydelicio = new Joystick(Constants.joydelicio_ID);
  private SimSubsystem SimSubSys;
  private VisionSimSubsystem VisSubSys;

  public RobotContainer() {

    if (RobotBase.isSimulation()) {
      SimSubSys = new SimSubsystem();
      VisSubSys = new VisionSimSubsystem();
    }

    DriveCommand DefaultDrive = new DriveCommand(SubSys, SimSubSys, joydelicio);
    SubSys.setDefaultCommand(DefaultDrive); // ALTERE AQUI PARA MUDAR O SUBSISTEMA
  }

  public void simulationPeriodic() {
    if (RobotBase.isSimulation() && VisSubSys != null && SimSubSys != null) {
      VisSubSys.updateSimPose(SimSubSys.getPose2D());
  }
}

  // private void configureBindings() {}
  
  public Command getAutonomousCommand() {
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new GoToPose(SubSys, 2, 2, 5),
      new TagFollower(SubSys, 0.4),
      new TagFollowerSim(SimSubSys, VisSubSys)
    );
    
    return auto;
    }
  }
