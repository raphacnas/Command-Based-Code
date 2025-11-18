package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends Command {

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private DriveSubsystem SubSys;
  private final double distance;
  Timer timer = new Timer();

  double tx, tv, ta, Rm, Lm;

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
    SubSys.setMotorSpeeds(Rm, Lm);
   
    if (timer.get() < 3){
      Rm = distance; Lm = distance;
    } else {
      TagFollower();
    }

    SmartDashboard();
  }

  public void TagFollower() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ta = limelight.getEntry("ta").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);

    // Ajustes finos
    double rot_percent = 0.01;     
    double fwd_percent = 0.2;      
    double targetArea = 5;  

    if (tv == 0) { 

      Rm = 0.25;
      Lm = -0.28; 

    } else {

      double rot = rot_percent * tx; 
      double forward = fwd_percent * (targetArea - ta);
    
      forward = Math.max(-0.6, Math.min(forward, 0.6));
      rot = Math.max(-0.4, Math.min(rot, 0.4));
    
      Lm = forward + rot;
      Rm = forward - rot;
    }
  }

  public void SmartDashboard() {
    SmartDashboard.putNumber("** - Tx", tx);
    SmartDashboard.putNumber("** - Ta", ta);
    SmartDashboard.putNumber("** - Tv", tv);
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
