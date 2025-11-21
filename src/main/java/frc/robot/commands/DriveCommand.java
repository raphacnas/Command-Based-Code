package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathFunctions.Calcs;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {

  private DriveSubsystem SubSys;
  private Joystick joydelicio;

  private final VictorSPX RMot1 = new VictorSPX(Constants.RMot1_ID);
  private final VictorSPX RMot2 = new VictorSPX(Constants.RMot2_ID);
  private final VictorSPX LMot1 = new VictorSPX(Constants.LMot1_ID);
  private final VictorSPX LMot2 = new VictorSPX(Constants.LMot2_ID);

  int POG;
  boolean a, b, x;
  double spdbutton, Lm, Rm, Ltrig, Rtrig, x1, x2, y1, y2;

  private final Calcs Calcs = new Calcs(x1, x2, y1, y2, Ltrig, Rtrig);
  
  /** Creates a new DefaultDriveCommand. */
  public DriveCommand(DriveSubsystem drive, Joystick joyzao) {
    
    this.SubSys = drive;
    this.joydelicio = joyzao;

    addRequirements(SubSys);
  }

  @Override
  public void initialize() {

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

    spdbutton = 1; Ltrig = 0; Rtrig = 0; Lm = 0; Rm = 0;
  }

  
  @Override
  public void execute() {
    // Called every time the scheduler runs while the command is scheduled.
    SmartDash();
    CalcButton();
    joyValues();
    
    if (POG != -1){
      double[] PovSpeeds = Calcs.CalcPOV(POG, spdbutton);

      Lm = PovSpeeds[0];
      Rm = PovSpeeds[1];

    } else if (Ltrig < -Constants.Deadzone && Rtrig <= Constants.Deadzone) {
      Ltrig = Calcs.CalcLTrig(Ltrig, Rtrig, spdbutton);

      Lm = Ltrig;
      Rm = Ltrig;
      

    } else if (Rtrig > Constants.Deadzone && Ltrig >= -Constants.Deadzone) {
      Rtrig = Calcs.CalcRTrig(Ltrig, Rtrig, spdbutton);

      Lm = Rtrig;
      Rm = Rtrig;

    } else {

      double[] MagAndSine = Calcs.CalcMagAndSine(-x1, x2, y1, -y2);
      double[] AnalogSpeeds = Calcs.CalcAnalogs(MagAndSine, spdbutton, -x1, y1, x2, -y2);

      if (AnalogSpeeds[0] != 0 || AnalogSpeeds[1] != 0) {
        Lm = AnalogSpeeds[0];
        Rm = AnalogSpeeds[1];
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

public void joyValues(){
  a = joydelicio.getRawButton(Constants.ButA_ID);
  b = joydelicio.getRawButton(Constants.ButB_ID);
  x = joydelicio.getRawButton(Constants.ButX_ID);

  this.x1 = joydelicio.getRawAxis(Constants.X1_ID);
  this.x2 = joydelicio.getRawAxis(Constants.X2_ID);
  this.y1 = joydelicio.getRawAxis(Constants.Y1_ID);
  this.y2 = joydelicio.getRawAxis(Constants.Y2_ID);

  this.Ltrig = joydelicio.getRawAxis(Constants.Ltrig_ID);
  this.Rtrig = joydelicio.getRawAxis(Constants.Rtrig_ID);

  POG = joydelicio.getPOV();
}
  
  public void CalcButton() {

    if(joydelicio.getRawButton(Constants.ButA_ID)) {
      spdbutton = 0.25;
    } if (joydelicio.getRawButton(Constants.ButB_ID)) {
      spdbutton = 0.5;
    } if (joydelicio.getRawButton(Constants.ButX_ID)) {
      spdbutton = 1;
    }
  }

  public void SmartDash() {

    SmartDashboard.putNumber("1 - Btn Spd", spdbutton);
    SmartDashboard.putBoolean("2 - btn A", a);
    SmartDashboard.putBoolean("3 - btn B", b);
    SmartDashboard.putBoolean("4 - btn X", x);
    SmartDashboard.putNumber("5 - POV", POG);
    SmartDashboard.putNumber("6 - LTrig", Ltrig);
    SmartDashboard.putNumber("7 - RTrig", Rtrig);
    SmartDashboard.putNumber("** - Left_X", x1);
    SmartDashboard.putNumber("** - Left_Y", y1);
    SmartDashboard.putNumber("** - Right_X", x2);
    SmartDashboard.putNumber("** - Right_Y", y2);
  }
}