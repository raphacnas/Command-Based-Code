package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPoseSim extends Command {

    private final DriveSubsystem drive;
    private final Pose2d target;

    public GoToPoseSim(DriveSubsystem drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
        addRequirements(drive);
    }

    @Override
    public void execute() {

        Pose2d current = drive.getPose2D();

        double errorX = target.getX() - current.getX();
        double errorY = target.getY() - current.getY();

        // distância até o alvo
        double distance = Math.sqrt(errorX*errorX + errorY*errorY);

        // ângulo atual e desejado
        double desiredHeading = Math.atan2(errorY, errorX);
        double headingError = desiredHeading - current.getRotation().getRadians();

        // Normaliza para [-pi, pi]
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // Controladores proporcionais
        double kFwd = 0.8;
        double kRot = 1.8;

        double forward = kFwd * distance;
        double rotation = kRot * headingError;

        // limitações
        forward = Math.max(-0.6, Math.min(0.6, forward));
        rotation = Math.max(-0.5, Math.min(0.5, rotation));

        // Differential drive
        double left = forward + rotation;
        double right = forward - rotation;

        drive.setMotorSpeeds(left, right);
    }

    @Override
    public boolean isFinished() {
        // Para quando estiver muito perto
        Pose2d current = drive.getPose2D();
        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        return Math.hypot(dx, dy) < 0.5; // 8 cm
    }
}
