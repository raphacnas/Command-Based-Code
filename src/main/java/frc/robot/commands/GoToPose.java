package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPose extends Command {

  private final DriveSubsystem SubSys;

  private final double targetX;
  private final double targetY;
  private final double targetRotDeg; // rotação final desejada

  // PID de distância
  private final PIDController distPID = new PIDController(1.0, 0, 0);

  // PID do ângulo
  private final PIDController anglePID = new PIDController(0.02, 0, 0);

  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable("limelight-front");

  public GoToPose(DriveSubsystem SubSys, double x, double y, double rotDegrees) {
    this.SubSys = SubSys;

    this.targetX = x;
    this.targetY = y;
    this.targetRotDeg = rotDegrees;

    anglePID.enableContinuousInput(-180, 180);

    addRequirements(SubSys);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double[] botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);

    if (botpose.length < 7) {
      SubSys.setMotorSpeeds(0, 0);
      return;
    }

    double curX = botpose[0];
    double curY = botpose[1];
    double curRot = botpose[5]; // yaw

    // Distância até o alvo
    double dx = targetX - curX;
    double dy = targetY - curY;
    double dist = Math.sqrt(dx * dx + dy * dy);

    // Ângulo desejado para ir até o ponto
    double desiredAngle = Math.toDegrees(Math.atan2(dy, dx));

    // PID de translação
    double forward = distPID.calculate(dist, 0);

    // PID de rotação para alinhar com o ponto
    double turn = anglePID.calculate(curRot, desiredAngle);

    // Se está perto do ponto final, troca para alinhar a rotação final desejada
    if (dist < 0.20) {
      turn = anglePID.calculate(curRot, targetRotDeg);
      forward = 0; // não avança mais
    }

    // Limites de segurança
    forward = clamp(forward, -0.5, 0.5);
    turn = clamp(turn, -0.5, 0.5);

    double left = forward + turn;
    double right = forward - turn;

    left = clamp(left, -1, 1);
    right = clamp(right, -1, 1);

    SubSys.setMotorSpeeds(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    SubSys.setMotorSpeeds(0, 0);
  }

  @Override
  public boolean isFinished() {

    double[] botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    if (botpose.length < 7) return false;

    double curX = botpose[0];
    double curY = botpose[1];
    double curRot = botpose[5];

    double dx = Math.abs(curX - targetX);
    double dy = Math.abs(curY - targetY);
    double drot = Math.abs(curRot - targetRotDeg);

    return dx < 0.05 && dy < 0.05 && drot < 3; // 5 cm e 3º
  }

  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
