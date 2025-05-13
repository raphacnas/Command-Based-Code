package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final VictorSPX RMot1 = new VictorSPX(3);
  private final VictorSPX RMot2 = new VictorSPX(4);
  private final VictorSPX LMot1 = new VictorSPX(1);
  private final VictorSPX LMot2 = new VictorSPX(2);

  double Rm, Lm;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RMot2.follow(RMot1);
    LMot2.follow(LMot1);

    RMot1.setInverted(true);
    LMot1.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    RMot1.set(ControlMode.PercentOutput, Rm);
    LMot1.set(ControlMode.PercentOutput, Lm);
  }
}