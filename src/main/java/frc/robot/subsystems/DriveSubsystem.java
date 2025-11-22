package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_VictorSPX RMot1 = new WPI_VictorSPX(Constants.RMot1_ID);
  private final WPI_VictorSPX RMot2 = new WPI_VictorSPX(Constants.RMot2_ID);
  private final WPI_VictorSPX LMot1 = new WPI_VictorSPX(Constants.LMot1_ID);
  private final WPI_VictorSPX LMot2 = new WPI_VictorSPX(Constants.LMot2_ID);

  double Rm, Lm;

  public DriveSubsystem() {

    RMot2.follow(RMot1);
    LMot2.follow(LMot1);

    RMot1.configNeutralDeadband(Constants.Deadzone);
    RMot2.configNeutralDeadband(Constants.Deadzone);
    LMot1.configNeutralDeadband(Constants.Deadzone);
    LMot2.configNeutralDeadband(Constants.Deadzone);

    RMot1.setInverted(false);
    LMot1.setInverted(true);
    RMot2.setInverted(InvertType.FollowMaster);
    LMot2.setInverted(InvertType.FollowMaster);

    RMot1.setNeutralMode(NeutralMode.Brake);
    RMot2.setNeutralMode(NeutralMode.Brake);
    LMot1.setNeutralMode(NeutralMode.Brake);
    LMot2.setNeutralMode(NeutralMode.Brake);
}

  public void setMotorSpeeds(double lSpeed, double rSpeed) {
    Lm = lSpeed;
    Rm = rSpeed;
  
    LMot1.set(ControlMode.PercentOutput, lSpeed);
    LMot2.set(ControlMode.PercentOutput, lSpeed);
    RMot1.set(ControlMode.PercentOutput, rSpeed);
    RMot2.set(ControlMode.PercentOutput, rSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("* - Lmot", Lm);
    SmartDashboard.putNumber("* - Rmot", Rm);
  }
}