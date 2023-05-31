

int limited(int a, float min, float max){
    if (a >= max){
        return max;
    }else if (a <= min){
        return min;
    }
    return a;
}

void pid_function(float Error, float P , float I, float D, float PrevError, float PrevIterm, float min, float max, float PIDReturn[]) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  Iterm = limited(Iterm, min, max);
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  PIDOutput = limited(PIDOutput, min, max);
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}