package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TagFollower extends Command {

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
  DriveSubsystem SubSys;
  private final double Rspeed;

  double tx, tv, ta, Rm, Lm;

  public TagFollower(DriveSubsystem SubSys, double Rspeed) {

    this.SubSys = SubSys;
    this.Rspeed = Rspeed;

    addRequirements(SubSys);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    tagFollower();
    SmartDashboard();

    SubSys.setMotorSpeeds(Lm, Rm);
  }

  public void tagFollower() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ta = limelight.getEntry("ta").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);

    // Ajustes finos
    double rot_percent = 0.01;     
    double fwd_percent = 0.2;      
    double targetArea = 1.25;  

    if (tv == 0){
      Rm = Rspeed; Lm = Rspeed; 
    } else { 
      if (ta >= targetArea) { 
        Rm = 0; Lm = 0; 
      } else {
  
        double rot = rot_percent * tx; 
        double forward = fwd_percent * (targetArea - ta);
      
        forward = Math.max(-0.6, Math.min(forward, 0.6));
        rot = Math.max(-0.4, Math.min(rot, 0.4));
      
        Lm = forward + rot;
        Rm = forward - rot;
      }
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