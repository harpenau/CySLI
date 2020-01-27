#ifndef PREDICTION_H
#define PREDICTION_H

double Kalman(double UnFV, double FR1, double *Pold);
double ApogeePrediction();
double Integrate(unsigned long PrevTime, unsigned long currTime, double Val);
double Derive(unsigned long OldTime, unsigned long time, double altPrev, double altRefine);
void Burnout();
void EndGame();
void UpdateData();

#endif