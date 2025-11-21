package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSimSubsystem;

public class TagFollowerCommand extends Command {

  private DriveSubsystem SubSys;
  private VisionSimSubsystem visionSim; 
  Timer timer = new Timer();

  double tx, tv, ta, Rm, Lm;

  public TagFollowerCommand(DriveSubsystem SubSys, VisionSimSubsystem visionSim) {
    this.SubSys = SubSys;
    this.visionSim = visionSim;

    addRequirements(SubSys);
}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    TagFollower();  
    SmartDashboard();
    SubSys.setMotorSpeeds(Lm, Rm);
  }

  public void TagFollower() {

    PhotonCamera cam = visionSim.getCamera();
    var result = cam.getLatestResult();

    if (!result.hasTargets()) {
      Lm = 0.3;
      Rm = -0.3;
      SubSys.setMotorSpeeds(Lm, Rm);
      return;
    }

    var best = result.getBestTarget();

    tx = best.getYaw();
    ta = best.getArea();

    // Ajustes finos
    double rot_percent = 0.01;     
    double fwd_percent = 0.5;      
    double targetArea = 1.25;  


    double rot = rot_percent * tx; 
    double forward = fwd_percent * (targetArea - ta);
  
    forward = Math.max(-0.6, Math.min(forward, 0.6));
    rot = Math.max(-0.4, Math.min(rot, 0.4));

    if (ta >= targetArea) {
      forward = 0;
    }
  
    Lm = forward + rot;
    Rm = forward - rot;
  }

  public void SmartDashboard() {
    SmartDashboard.putNumber("** - Tx", tx);
    SmartDashboard.putNumber("** - Ta", ta);
  }
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
