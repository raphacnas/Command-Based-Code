package frc.robot.MathFunctions;

import frc.robot.Constants;

public class Calcs {
    private boolean LeftAnalog = true;
    private boolean RightAnalog = false;

    public double[] CalcMagAndSine(double x1, double x2, double y1, double y2) {
        double MagAndSine[] = new double[4];
        
        MagAndSine[0] = Math.hypot(x1, y1); // Mag 1
        MagAndSine[1] = Math.hypot(x2, y2); // Mag 2 
        
        MagAndSine[0] = Math.max(-1, Math.min(MagAndSine[0], 1));
        MagAndSine[1] = Math.max(-1, Math.min(MagAndSine[1], 1));
        
        MagAndSine[2] = y1 / MagAndSine[0]; // Seno 1
        MagAndSine[3] = y2 / MagAndSine[1]; // Seno 2
        
        return MagAndSine;
    }

    public double CalcLTrig(double Ltrig, double Rtrig, double spdbutton) {
        return (Ltrig <= Constants.Deadzone && Rtrig >= Constants.Deadzone) ? Ltrig * spdbutton : 0;
    }

    public double CalcRTrig(double Ltrig, double Rtrig, double spdbutton) {
        return (Rtrig <= Constants.Deadzone && Ltrig >= Constants.Deadzone) ? Rtrig * spdbutton : 0;
    }
  
    public double[] CalcAnalogs(double[] MagAndSine, double spdbutton, double x1, double y1, double x2, double y2) {
        double[] MotSpeed = new double[2];
        
        if (MagAndSine[0] > 0.04) {
            LeftAnalog = true;
            RightAnalog = false;
        } 
        else if (MagAndSine[1] > 0.04) {
            RightAnalog = true;
            LeftAnalog = false;
        }
        
        if (LeftAnalog) {
            if (x1 >= 0 && y1 >= 0) {
                MotSpeed[0] = MagAndSine[0] * spdbutton;
                MotSpeed[1] = MagAndSine[2] * MagAndSine[0] * spdbutton;
            }
            else if (x1 < 0 && y1 >= 0) {
                MotSpeed[0] = (2 * MagAndSine[2] - 1) * MagAndSine[0] * spdbutton;
                MotSpeed[1] = MagAndSine[0] * spdbutton;
            }
            else if (x1 >= 0 && y1 < 0) {
                MotSpeed[0] = -MagAndSine[0] * spdbutton;
                MotSpeed[1] = (2 * MagAndSine[2] + 1) * MagAndSine[0] * spdbutton;
            }
            else if (x1 < 0 && y1 < 0) {
                MotSpeed[0] = MagAndSine[2] * MagAndSine[0] * spdbutton;
                MotSpeed[1] = -MagAndSine[0] * spdbutton;
            }
        } 
        else if (RightAnalog) {
            if (x2 >= 0 && y2 >= 0) {
                MotSpeed[0] = -MagAndSine[3] * MagAndSine[1] * spdbutton;
                MotSpeed[1] = -MagAndSine[1] * spdbutton;
            }
            else if (x2 < 0 && y2 >= 0) {
                MotSpeed[0] = -MagAndSine[1] * spdbutton;
                MotSpeed[1] = -MagAndSine[3] * MagAndSine[1] * spdbutton;
            }
            else if (x2 >= 0 && y2 < 0) {
                MotSpeed[0] = -MagAndSine[3] * MagAndSine[1] * spdbutton;
                MotSpeed[1] = MagAndSine[1] * spdbutton;
            }
            else if (x2 < 0 && y2 < 0) {
                MotSpeed[0] = MagAndSine[1] * spdbutton;
                MotSpeed[1] = -MagAndSine[3] * MagAndSine[1] * spdbutton;
            }
        }
        
        return MotSpeed;
    }

    public double[] CalcPOV(int POG, double spdbutton) {
        double[] MotSpeed = new double[2];
        
        switch (POG) {
            case 0:
                MotSpeed[0] = 1;
                MotSpeed[1] = 1;
                break;
            case 45:
                MotSpeed[0] = 1;
                MotSpeed[1] = 0;
                break;
            case 90:
                MotSpeed[0] = 1;
                MotSpeed[1] = -1;
                break;
            case 135:
                MotSpeed[0] = -1;
                MotSpeed[1] = 0;
                break;
            case 180:
                MotSpeed[0] = -1;
                MotSpeed[1] = -1;
                break;
            case 225:
                MotSpeed[0] = 0;
                MotSpeed[1] = -1;
                break;
            case 270:
                MotSpeed[0] = -1;
                MotSpeed[1] = 1;
                break;
            case 315:
                MotSpeed[0] = 0;
                MotSpeed[1] = 1;
                break;
            default:
                MotSpeed[0] = 0;
                MotSpeed[1] = 0;

        } MotSpeed[0] *= spdbutton; MotSpeed[1] *= spdbutton;
        
        return MotSpeed;
    }
}