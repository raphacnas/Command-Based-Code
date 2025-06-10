package frc.robot.MathFunctions;

import frc.robot.Constants;

public class Calcs {

  public double[] CalcMagAndSine (double x1, double x2, double y1, double y2) {

    // 0 = Mag1, 1 = Mag2, 2 = Sen1, 3 = Sen2
    double MagAndSine[] = new double[4];

    MagAndSine[0] = Math.hypot(x1, y1);
    MagAndSine[1] = Math.hypot(x2, y2);

    MagAndSine[0] = Math.max(-1, Math.min(MagAndSine[0], 1));
    MagAndSine[1]  = Math.max(-1, Math.min(MagAndSine[1], 1));

    MagAndSine[2] = y1 / MagAndSine[0];
    MagAndSine[3] = y2 / MagAndSine[1];

    return MagAndSine;
  }

  public double CalcLTrig (double Ltrig, double Rtrig, double spdbutton) {
    if (Ltrig > Constants.Deadzone && Rtrig <= Constants.Deadzone) {
      Ltrig *= spdbutton;
    } return Ltrig;
  }

  public double CalcRTrig (double Ltrig, double Rtrig, double spdbutton) {
    if (Rtrig > Constants.Deadzone && Ltrig <= Constants.Deadzone) {
      Rtrig *= spdbutton;
    } return Rtrig;
  }
  
  public double[] CalcAnalogs (double[] MagAndSine, double spdbutton, double x1, double y1, double x2, double y2) {

    // 0 = Left Motor, 1 = Right Motor
    double[] MotSpeed = new double[2];
    boolean analog1, analog2;
    
    if (MagAndSine[0] != 0) {

      analog1 = true;
      analog2 = false;

      if (!analog2) { 
        // QUADRANTE 1
        if (x1 >= 0 && y1 >= 0) {
          MotSpeed[0] =  MagAndSine[0] * spdbutton;
          MotSpeed[1] = (2 * MagAndSine[2] - 1) * MagAndSine[0] * spdbutton;
        }
        // QUADRANTE 2
        else if (x1 < 0 && y1 >= 0) {
          MotSpeed[0] = (2 * MagAndSine[2] - 1) * MagAndSine[0] * spdbutton;
          MotSpeed[1] =  MagAndSine[0] * spdbutton;
        }
        // QUADRANTE 3
        else if (x1 >= 0 && y1 < 0) {
          MotSpeed[0] =  -MagAndSine[0] * spdbutton;
          MotSpeed[1] = (2 * MagAndSine[2] + 1) * MagAndSine[0] * spdbutton;
        }
        // QUADRANTE 4
        else if (x1 < 0 && y1 < 0) {
          MotSpeed[0] = (2 * MagAndSine[2] + 1) * MagAndSine[0] * spdbutton;
          MotSpeed[1] = -MagAndSine[0] * spdbutton;
        } 

      }
    } if (MagAndSine[1] != 0) {

      analog1 = false;
      analog2 = true;

      if (!analog1) { 

        // QUADRANTE 1
        if (x2 >= 0 && y2 >= 0) {
          MotSpeed[0] = -MagAndSine[1] * spdbutton;
          MotSpeed[1] = (2 * MagAndSine[3] + 1) * MagAndSine[1] * spdbutton;
        }
        // QUADRANTE 2
        else if (x2 < 0 && y2 >= 0) {
          MotSpeed[0] = (2 * MagAndSine[3] + 1) * MagAndSine[1] * spdbutton; 
          MotSpeed[1] = -MagAndSine[1] * spdbutton;
        }
        // QUADRANTE 3
        else if (x2 >= 0 && y2 < 0) {
          MotSpeed[0] = MagAndSine[1] * spdbutton;
          MotSpeed[1] = (-2 * MagAndSine[3] - 1) * MagAndSine[1] * spdbutton;
        }
        // QUADRANTE 4
        else if (x2 < 0 && y2 < 0) {
          MotSpeed[0] = (-2 * MagAndSine[3] - 1) * MagAndSine[1] * spdbutton;
          MotSpeed[1] = MagAndSine[1] * spdbutton; 
        }
      }
    } return MotSpeed;
  }

  public double[] CalcPOV (int POG, double spdbutton) {

    double[] MotSpeed = new double[2];
    
    switch (POG) {
      case 0:
        MotSpeed[1] = 0.40;
        MotSpeed[0] = 0.40;
        break;
      case 45:
        MotSpeed[1] = 0;
        MotSpeed[0] = 0.40;
        break;
      case 90:
        MotSpeed[1] = -0.40;
        MotSpeed[0] = 0.40;
        break;
      case 135:
        MotSpeed[1] = -0.40;
        MotSpeed[0] = 0;
        break;
      case 180:
        MotSpeed[1] = -0.40;
        MotSpeed[0] = -0.40;
        break;
      case 225:
        MotSpeed[1] = 0;
        MotSpeed[0] = -0.40;
        break;
      case 270:
        MotSpeed[1] = -0.40;
        MotSpeed[0] = 0.40;
        break;
      case 315:
        MotSpeed[1]= 0.40;
        MotSpeed[0] = 0;
        break;

      default:
      case -1:
        MotSpeed[1] = 0;
        MotSpeed[0] = 0;
    } return MotSpeed;
  }
}