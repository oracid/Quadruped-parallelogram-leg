//FourBarParallelogram-2 is a quadruped parallelogram leg.
#include <Servo.h>
const int nbSrv=2;          // Number of servos
Servo Srv[nbSrv];           // Servos table
int SrvOn[nbSrv]={1,1};     // Servos table on/off. 1=on, 0=off
int Speed=20;               // here you can change the speed
int errL=0, errR=4;         // Here you need to correct the 0° position of your servos
// Here you need to correct the Point(0,0) in map instruction
// The extreme values of x must be determined with the HLine () function
int LmapX=-135, RmapX=135;

void  setup() {
  Serial.begin(9600);
  pinMode(0,INPUT_PULLUP);                    //start button attachment

  Srv[0].attach(4); Srv[0].write(80+errL);    // left  servo attachement and position setting
  Srv[1].attach(8); Srv[1].write(80+errR);    // right servo attachement and position setting

  Serial.print("\n\t To start, click on the Start button");
  while( digitalRead(0) );  delay(400);       // waiting for start button pressed 
  Serial.print("\n\t Started");
}

void  loop(){
  Square(-25,25,10,30);         // draw a square from x=-25, y=10 to x=25, y=30

// examples of basic function by uncommenting one line only
//  Point(0,10);delay(500);       // go to x, y position. delay() is optional
//  Torque();                     // Torque testing function
//  VLine(1,30,0);                // draw a vertical line from y)=1 to y=30 with x=0
//  HLine(-30,30,10);             // draw an horizontal line from x=-30 to x=30 and y=10
}

void Point(int x, int y){
  x=map(x,-100,100,LmapX,RmapX);
  InverseKinematics(x,y,Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
}

void VLine(int Ay, int By, int x){
  if(Ay<By){for (int i=Ay; i<=By;i++) {Point(x,i);}
  }else{for (int i=Ay; i>=By;i--) {Point(x,i);}}
}

void HLine(int Ax, int Bx, int y){
  if(Ax<Bx){for (int i=Ax; i<=Bx;i++) {Point(i,y);}
  }else{for (int i=Ax; i>=Bx;i--) {Point(i,y);}}
}

void Square(int Ax, int Bx, int Ay, int By){
  HLine(Ax, Bx, Ay);
  VLine(Ay, By, Bx);
  HLine(Bx, Ax, By);
  VLine(By, Ay, Ax);
}
void Torque(){
  VLine(1,33,0);delay(300);
  VLine(33,1,0);delay(300);
}

void InverseKinematics(int Px, int Py, Servo srvL, Servo srvR, int srvLon, int srvRon){
  float A1x=0, A1y=144, A2x=0, A2y=144;               // Values of servos positions
  float a1=32, c1=112, a2=112, c2=32;                 // Values of legs sides lengths

  float d=A1y-Py, e=Px;                               //value of right triangle sides length
  float b=sqrt((d*d)+(e*e));                          //value of right triangle hypotenuse
  float S=acos(d/b);  if(e<0)S=(-S);                  //value of S angle of right triangle
  float A12=acos(((b*b)+(c1*c1)-(a1*a1))/(2*b*c1));   //value of the adjacent left angle to S
  float A22=acos(((b*b)+(c2*c2)-(a2*a2))/(2*b*c2));   //value of the adjacent right angle to S
  float A11=(PI/2)-A12+S;                             //value of the target left angle 
  float A21=(PI/2)-A22-S;                             //value of the target right angle

  int S1=round(A11*57.296);                           //left servo angle
  int S2=round(A21*57.296);                           //right servo angle
/* DEBUG
  Serial.print("\n\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);
  Serial.print("\n\t d=");Serial.print(d);Serial.print("\t\t e=");Serial.print(e);Serial.print("\t\t b=");Serial.print(b);Serial.print("\t\t S=");Serial.print(S*57.296);
  Serial.print("\n\t A11=");Serial.print(A11*57.296);Serial.print("\t\t A12=");Serial.print(A12*57.296);Serial.print("\t\t A22=");Serial.print(A22*57.296);Serial.print("\t\t A21=");Serial.print(A21*57.296);
  Serial.print("\n\t Result of calculations, angles of the servos");
  Serial.print("\n\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°");
*/
  if ( b>(a1+c1) ){
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print("\t b=");Serial.print(b);Serial.print(" > ");
    Serial.print(a1+c1);Serial.print(" is too long. Target impossible to reach   !!!!!");
    return;
  }
  if (S1<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1<0° is not reachable   !!!!!");
    return;
  }
  if (S2<0){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2<0° is not reachable   !!!!!");
    return;
  }
  if (S1>140){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S1>140° is not reachable   !!!!!");
    return;
  }
  if (S2>140){
    Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\t angle S2>140° is not reachable   !!!!!");
    return;
  }
//  Serial.print("\t executed command");
  if (srvLon) srvL.write(S1+errL);        // set target Left servo position if servo switch is On
  if (srvRon) srvR.write(S2+errR);        // set target Right servo position if servo switch is On
  delay(Speed);
}
