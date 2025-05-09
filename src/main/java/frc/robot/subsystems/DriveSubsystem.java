package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveSubsystem extends SubsystemBase {

  private final VictorSPX RMot1 = new VictorSPX(3);
  private final VictorSPX RMot2 = new VictorSPX(4);
  private final VictorSPX LMot1 = new VictorSPX(1);
  private final VictorSPX LMot2 = new VictorSPX(2);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RMot2.follow(RMot1);
    LMot2.follow(LMot1);

    RMot1.setInverted(true);
    LMot1.setInverted(false);

    RMot1.setSelectedSensorPosition(0);
    LMot1.setSelectedSensorPosition(0);


    RMot1.configNeutralDeadband(0.04);
    LMot1.configNeutralDeadband(0.04);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
