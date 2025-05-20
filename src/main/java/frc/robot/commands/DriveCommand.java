package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathFunctions.Calcs;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {

  private DriveSubsystem SubSys;
  private final Calcs Calcs = new Calcs();
  private Joystick joydelicio;

  private final VictorSPX RMot1 = new VictorSPX(Constants.RMot1_ID);
  private final VictorSPX RMot2 = new VictorSPX(Constants.RMot2_ID);
  private final VictorSPX LMot1 = new VictorSPX(Constants.LMot1_ID);
  private final VictorSPX LMot2 = new VictorSPX(Constants.LMot2_ID);

  int POG;
  boolean a, b, x, toggleA,toggleB, toggleX;
  double spdbutton, Lm, Rm, Ltrig, Rtrig, x1, x2, y1, y2;
  
  /** Creates a new DefaultDriveCommand. */
  public DriveCommand(DriveSubsystem drive, Joystick joyzao) {
    
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

    RMot1.configNeutralDeadband(Constants.Deadzone);
    LMot1.configNeutralDeadband(Constants.Deadzone);

    spdbutton = 1; Ltrig = 0; Rtrig = 0; Lm = 0; Rm = 0;
  }

  
  @Override
  public void execute() {
    // Called every time the scheduler runs while the command is scheduled.

    a = joydelicio.getRawButton(Constants.ButA_ID);
    b = joydelicio.getRawButton(Constants.ButB_ID);
    x = joydelicio.getRawButton(Constants.ButX_ID);

    x1 = joydelicio.getRawAxis(Constants.X1_ID);
    x2 = joydelicio.getRawAxis(Constants.X2_ID);
    y1 = -joydelicio.getRawAxis(Constants.Y1_ID);
    y2 = -joydelicio.getRawAxis(Constants.Y2_ID);

    Ltrig = -joydelicio.getRawAxis(Constants.Ltrig_ID);
    Rtrig = joydelicio.getRawAxis(Constants.Rtrig_ID);

    POG = joydelicio.getPOV();

    Calcs.CalcMagAndSine(x1, x2, y1, y2);
    CalcButton();

    if (POG != -1){
      double[] PovSpeeds = Calcs.CalcPOV(POG, spdbutton);

      Lm = PovSpeeds[0];
      Rm = PovSpeeds[1];

    } else {

      double[] MagAndSine = Calcs.CalcMagAndSine(x1, x2, y1, y2);
      double[] AnalogSpeeds = Calcs.CalcAnalogs(MagAndSine, spdbutton, x1, y1, x2, y2);

      if (AnalogSpeeds[0] != 0 || AnalogSpeeds[1] != 0) {
        Lm = AnalogSpeeds[0];
        Rm = AnalogSpeeds[1];
      } else {
        Lm = Calcs.CalcLTrig(Ltrig, Rtrig, spdbutton);
        Rm = Calcs.CalcRTrig(Ltrig, Rtrig, spdbutton);
      }
    } SubSys.setMotorSpeeds(Lm, Rm);    
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
    if (joydelicio.getRawButtonPressed(Constants.ButA_ID)) toggleA = !toggleA;
    if (joydelicio.getRawButtonPressed(Constants.ButB_ID)) toggleB = !toggleB;
    if (joydelicio.getRawButtonPressed(Constants.ButX_ID)) toggleX = !toggleX;

    spdbutton = toggleA ? 0.25 : toggleB ? 0.5 : toggleX ? 1.0 : 1.0;
  }

}