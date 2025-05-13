package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {

  private DriveSubsystem SubSys;
  private Joystick joydelicio;

  private final VictorSPX RMot1 = new VictorSPX(3);
  private final VictorSPX RMot2 = new VictorSPX(4);
  private final VictorSPX LMot1 = new VictorSPX(1);
  private final VictorSPX LMot2 = new VictorSPX(2);

  int POG;
  boolean a, b, x, analog1, analog2, toggleA,toggleB, toggleX;
  double spdbutton, Lm, Rm, Ltrig, Rtrig, mag, mag2, sen, sen2, x1, x2, y1, y2;
  
  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DriveSubsystem drive, Joystick joyzao) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.SubSys = drive;
    this.joydelicio = joyzao;

    addRequirements(SubSys);
  }

  @Override
  public void initialize() {
    // Called when the command is initially scheduled.

    RMot2.follow(RMot1);
    LMot2.follow(LMot1);

    RMot1.setInverted(true);
    LMot1.setInverted(false);

    RMot1.configNeutralDeadband(0.04);
    LMot1.configNeutralDeadband(0.04);

    spdbutton = 1; Ltrig = 0; Rtrig = 0; Lm = 0; Rm = 0;
  }

  
  @Override
  public void execute() {
    // Called every time the scheduler runs while the command is scheduled.

    a = joydelicio.getRawButton(1);
    b = joydelicio.getRawButton(2);
    x = joydelicio.getRawButton(3);

    x1 = joydelicio.getRawAxis(0);
    x2 = joydelicio.getRawAxis(4);
    y1 = joydelicio.getRawAxis(1);
    y2 = joydelicio.getRawAxis(5);

    Ltrig = -joydelicio.getRawAxis(3);
    Rtrig = joydelicio.getRawAxis(2);

    mag = Math.hypot(x1, y1);
    mag2 = Math.hypot(x2, y2);

    mag = Math.max(-1, Math.min(mag, 1));
    mag2  = Math.max(-1, Math.min(mag2, 1));

    sen = y1 / mag;
    sen2 = y2 / mag2;

    POG = joydelicio.getPOV();

    CalcButton();
    CalcLTrig();
    CalcRTrig();
    CalcAnalog1();
    CalcAnalog2();
    CalcPOV();

    if (!analog1 && !analog2) {
      Lm = 0;
      Rm = 0;
    }

    RMot1.set(ControlMode.PercentOutput, Rm);
    LMot1.set(ControlMode.PercentOutput, Lm);
  }

//////////////////////////////////////////////////////////////
  @Override
  public void end(boolean interrupted) {
    // Called once the command ends or is interrupted.
  }
  
  @Override
  public boolean isFinished() {
    // Returns true when the command should end.
    return false;
  }
//////////////////////////////////////////////////////////////


  public void CalcButton() {
    if (joydelicio.getRawButtonPressed(1)) toggleA = !toggleA;
    if (joydelicio.getRawButtonPressed(2)) toggleB = !toggleB;
    if (joydelicio.getRawButtonPressed(3)) toggleX = !toggleX;

    spdbutton = toggleA ? 0.25 : toggleB ? 0.5 : toggleX ? 1.0 : 1.0;
  }

  public void CalcLTrig() {
    if (Ltrig > 0.04 && Rtrig <= 0.04) {
      Ltrig *= spdbutton;
    }
  }

  public void CalcRTrig() {
    if (Rtrig > 0.04 && Ltrig <= 0.04) {
      Rtrig *= spdbutton;
    }
  }
  
  public void CalcAnalog1() {
    if (mag != 0 && !analog2) { 

      analog1 = true;
      analog2 = false;

      // QUADRANTE 1
      if (x1 >= 0 && y1 >= 0) {
        Lm =  mag * spdbutton;
        Rm = (2 * sen - 1) * mag * spdbutton;
      }
      // QUADRANTE 2
      else if (x1 < 0 && y1 >= 0) {
        Lm = (2 * sen - 1) * mag * spdbutton;
        Rm =  mag * spdbutton;
      }
      // QUADRANTE 3
      else if (x1 >= 0 && y1 < 0) {
        Lm =  -mag * spdbutton;
        Rm = (2 * sen + 1) * mag * spdbutton;
      }
      // QUADRANTE 4
      else if (x1 < 0 && y1 < 0) {
        Lm = (2 * sen + 1) * mag * spdbutton;
        Rm = -mag * spdbutton;
      }
    }
  }

  public void CalcAnalog2(){
    if (mag2 != 0 && !analog1) { 

      analog1 = false;
      analog2 = true;

      // QUADRANTE 1
      if (x2 >= 0 && y2 >= 0) {
        Lm = -mag2 * spdbutton;
        Rm = (2 * sen2 + 1) * mag2 * spdbutton;
      }
      // QUADRANTE 2
      else if (x2 < 0 && y2 >= 0) {
        Lm = (2 * sen2 + 1) * mag2 * spdbutton; 
        Rm = -mag2 * spdbutton;
      }
      // QUADRANTE 3
      else if (x2 >= 0 && y2 < 0) {
        Lm = mag2 * spdbutton;
        Rm = (-2 * sen2 - 1) * mag2 * spdbutton;
      }
      // QUADRANTE 4
      else if (x2 < 0 && y2 < 0) {
        Lm = (-2 * sen2 - 1) * mag2 * spdbutton;
        Rm = mag2 * spdbutton; 
      }
    }
  }

  public void CalcPOV() {
      switch (POG) {
        case 0:
          Rm = 0.40;
          Lm = 0.40;
          break;
        case 45:
          Rm = 0;
          Lm = 0.40;
          break;
        case 90:
          Rm = -0.40;
          Lm = 0.40;
          break;
        case 135:
          Rm = -0.40;
          Lm = 0;
          break;
        case 180:
          Rm = -0.40;
          Lm = -0.40;
          break;
        case 225:
          Rm = 0;
          Lm = -0.40;
          break;
        case 270:
          Rm = -0.40;
          Lm = 0.40;
          break;
        case 315:
          Rm = 0.40;
          Lm = 0;
          break;
  
        default:
        case -1:
         Rm = 0;
         Lm = 0;
    }
  }

}