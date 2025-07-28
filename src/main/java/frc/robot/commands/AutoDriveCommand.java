package frc.robot.commands;

import java.util.TooManyListenersException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends Command {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private DriveSubsystem SubSys;
  private final double distance;
  Timer timer = new Timer();

  double tx, ty, ta;

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
      limelight();
    }
  }

  public void limelight() {
    tx = table.getEntry("tx").getDouble(0.0);
    ty = table.getEntry("ty").getDouble(0.0);
    ta = table.getEntry("ta").getDouble(0.0);

    SmartDashboard.putNumber("*** - Tx", tx);
    SmartDashboard.putNumber("*** - Ty", ty);
    SmartDashboard.putNumber("*** - Ta", ta);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
