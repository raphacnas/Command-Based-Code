package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final VictorSPX RMot1 = new VictorSPX(Constants.RMot1_ID);
  private final VictorSPX RMot2 = new VictorSPX(Constants.RMot2_ID);
  private final VictorSPX LMot1 = new VictorSPX(Constants.LMot1_ID);
  private final VictorSPX LMot2 = new VictorSPX(Constants.LMot2_ID);

  double Rm, Lm;

  /** Runs when initialized. */
  public DriveSubsystem() {

    RMot1.configNeutralDeadband(Constants.Deadzone);
    RMot2.configNeutralDeadband(Constants.Deadzone);
    LMot1.configNeutralDeadband(Constants.Deadzone);
    LMot2.configNeutralDeadband(Constants.Deadzone);

    RMot1.setInverted(true);
    RMot2.setInverted(true);
    LMot1.setInverted(false);
    LMot2.setInverted(false);

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