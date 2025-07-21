package frc.robot.commands;

import java.util.TooManyListenersException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends Command {

  private DriveSubsystem SubSys;
  private final double distance;
  Timer timer = new Timer();

  public AutoDriveCommand(DriveSubsystem SubSys, double distance) {

    this.SubSys = SubSys;
    this.distance = distance;

    addRequirements(SubSys);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    timer.start();

    if (timer.get() < 3){
      SubSys.setMotorSpeeds(distance, distance);
    } else {
      SubSys.setMotorSpeeds(0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
