package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSimSubsystem extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;

  // Transformo que posiciona a “afonsoCamera” no robô
  private final Transform3d cameraTransform =
      new Transform3d(
          new Translation3d(-0.20, 0, 0.40),   // X,Y,Z em metros
          new Rotation3d(0.0, Math.toRadians(25), 0.0) // Pitch 25°
      );

  public VisionSimSubsystem() {

    camera = new PhotonCamera("afonsoCamera");

    if (RobotBase.isSimulation()) {

      // Propriedades da câmera simulada
      SimCameraProperties props = new SimCameraProperties();
      props.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      props.setFPS(90);
      props.setAvgLatencyMs(30);
      props.setLatencyStdDevMs(5);

      // Layout oficial do Reefscape 2025
      AprilTagFieldLayout layout =
          AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

      // Cria o sistema de visão
      visionSim = new VisionSystemSim("VisionSimulation");

      // Adiciona as tags
      visionSim.addAprilTags(layout);

      // Cria a câmera simulada e adiciona ao sistema
      cameraSim = new PhotonCameraSim(camera, props);
      visionSim.addCamera(cameraSim, cameraTransform);
    }
  }

  /**
   * Atualiza a simulação usando a pose atual do robô.
   * Esse método deve ser chamado pelo RobotContainer.
   */
  public void updateSimPose(edu.wpi.first.math.geometry.Pose2d pose) {
    if (RobotBase.isSimulation() && visionSim != null) {
      visionSim.update(pose);
    }
  }

  /** Getter para permitir que comandos acessem a câmera. */
  public PhotonCamera getCamera() {
    return camera;
  }

  @Override
  public void periodic() {
    // Apenas para rodar lógica futura se necessário
  }
}
