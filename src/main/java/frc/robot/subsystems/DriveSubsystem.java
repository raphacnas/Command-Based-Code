package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final Encoder afonsoR = new Encoder(1, 2, false);
  private final Encoder afonsoL = new Encoder(3, 4, true);

  private EncoderSim afonsoRSIM;
  private EncoderSim afonsoLSIM;

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  public ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

  public DifferentialDrivetrainSim driveSim;

  private final NetworkTableEntry poseEntry;
  private final Field2d field = new Field2d();

  private final DifferentialDriveOdometry odometry;

  private final WPI_VictorSPX RMot1 = new WPI_VictorSPX(Constants.RMot1_ID);
  private final WPI_VictorSPX RMot2 = new WPI_VictorSPX(Constants.RMot2_ID);
  private final WPI_VictorSPX LMot1 = new WPI_VictorSPX(Constants.LMot1_ID);
  private final WPI_VictorSPX LMot2 = new WPI_VictorSPX(Constants.LMot2_ID);

  double Rm, Lm;

  /** Runs when initialized. */
  public DriveSubsystem() {
    // Inicialize a odometria primeiro
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), afonsoL.getDistance(), afonsoR.getDistance());

    // Agora você pode chamar resetOdometry
    
    resetOdometry(new Pose2d());

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

    gyro.reset();

    if (RobotBase.isSimulation()) {
        afonsoL.setDistancePerPulse((Math.PI * 0.06) / 2048); // Meters per pulse
        afonsoR.setDistancePerPulse((Math.PI * 0.06) / 2048); // Meters per pulse

        afonsoLSIM = new EncoderSim(afonsoL);
        afonsoRSIM = new EncoderSim(afonsoR);
        gyroSim = new ADXRS450_GyroSim(gyro);

        driveSim = DifferentialDrivetrainSim.createKitbotSim(
            DifferentialDrivetrainSim.KitbotMotor.kDualMiniCIMPerSide,
            DifferentialDrivetrainSim.KitbotGearing.k10p71,
            DifferentialDrivetrainSim.KitbotWheelSize.kSixInch, null);
    }

    SmartDashboard.putData("Field", field);
    poseEntry = NetworkTableInstance.getDefault().getTable("Field").getEntry("RobotPose");
}
  
@Override
  public void simulationPeriodic() { 
    if (driveSim!=null) {
      driveSim.setInputs(RMot1.get() * 12, LMot1.get() * 12);
      driveSim.update(0.02);

      afonsoLSIM.setDistance(driveSim.getLeftPositionMeters());
      afonsoRSIM.setDistance(driveSim.getRightPositionMeters());

      afonsoLSIM.setRate(driveSim.getLeftVelocityMetersPerSecond());
      afonsoRSIM.setRate(driveSim.getRightVelocityMetersPerSecond());

      gyroSim.setAngle(driveSim.getHeading().getDegrees());
    }
  }

  public Pose3d getPose3D(){
    return new Pose3d(
      new Translation3d(getPose2D().getX(), getPose2D().getY(), 0),
      new Rotation3d(0.0, 0.0, getHeading().getRadians())
    );
  }

  public Pose2d getPose2D(){
    return odometry.getPoseMeters();
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  public void resetEncoders() {
    afonsoL.reset();
    afonsoR.reset();
  }

  // public void setMotorSpeeds(double lSpeed, double rSpeed) {
  //   Lm = lSpeed;
  //   Rm = rSpeed;
  //
  //   LMot1.set(ControlMode.PercentOutput, lSpeed);
  //   LMot2.set(ControlMode.PercentOutput, lSpeed);
  //   RMot1.set(ControlMode.PercentOutput, rSpeed);
  //   RMot2.set(ControlMode.PercentOutput, rSpeed);
  // }

  public void setMotorSpeeds (double lSpeed, double rSpeed) {
    LMot1.set(lSpeed);
    RMot1.set(rSpeed);

    Logger.recordOutput("Drive/LeftSpeed", lSpeed);
    Logger.recordOutput("Drive/RightSpeed", rSpeed);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    gyro.reset();
    odometry.resetPosition(getHeading(),afonsoL.getDistance(), afonsoR.getDistance(),pose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("* - Lmot", Lm);
    SmartDashboard.putNumber("* - Rmot", Rm);

    odometry.update(getHeading(), afonsoL.getDistance(), afonsoR.getDistance());

    Pose2d pose = odometry.getPoseMeters();
    Logger.recordOutput("Drive/pose", new Pose2d(pose.getX(), pose.getY(), pose.getRotation()));
    field.setRobotPose(pose);

    poseEntry.setDoubleArray(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
     Pose3d pose3d = new Pose3d(
      pose.getX(),
      pose.getY(),
      0,
      new Rotation3d(0, 0, pose.getRotation().getRadians())
     );
     if (RobotBase.isSimulation()){
      Logger.recordOutput("Sim/RobotPose", pose3d);
     } 
  }
}