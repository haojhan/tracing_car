#include <MsTimer2.h>
#include <Servo.h>
/*     theta=90
=  -----    
= |     |----->y, theta=0
=  -----|
        v  x, theta=-90
*/
/*    ------------
 *  |      ^x       |
 * |   y<--|         |
 *  |               |
 *    -------------
 *    
 */
//---------------foot declare-----------
const byte IRsize=5;
byte irf[IRsize] = {A15, A1, A2, A3, A4};//left to right
byte irb[IRsize] = {A5, A6, A7, A8, A9};//left to right
byte wr = A10, wl = A11;
byte steerfoot=3;
byte fanfoot=6;
byte brake_r_foot=4;
byte brake_l_foot=5;
byte track1=8; //add wire to decide
const byte n_us=3;
const byte trigPin[n_us] = {22, 24, 26};
const byte echoPin[n_us] = {18, 19, 20};
//-----------function optional para declare------------
void IR_update(bool printout=false);
//-------------parameter declare------------
//car size parameter
float w_delta=0.07;//0.144
float w_mean=60.192;
float wheel_diameter_r = w_mean-w_delta;//mm
float wheel_diameter_l = w_mean+w_delta;
float wheel_space = 161.84;//mm
float car_length =238;//mm . IR to IR 
float middle_to_wheel=178;//mm
float wheel_to_ir=66;
float IR_space=15;//mm

//en parameter
int encoder_threshold_left_high=700;
int encoder_threshold_left_low=200;
int encoder_threshold_right_high=700;
int encoder_threshold_right_low=200;
byte en_counter_len=2;
float theta_ir_bias=-0.01;

//us parameter
float us_threshold=800;
float us_side_thresh=1000;
float us_side_threshold=250;

//servo parameter
int br=0;//for setup
int bl=180;
int steer_zero=128;
int steer_max=149;
int steer_min=107;
int b_max=170;
int b_min=10;
int fan_max=1150;
int fan_min=1000;

//speed base ini parameter
int fanbase00=1100;
int fanbase0=fanbase00;
int fanbase=fanbase00;
int brakebase=100;
int steerbase=steer_zero;
float min_speed=0.;//mm


//trace parameter

float theta_ini=2*PI-3.*7.2/180*PI;
float s=2000*PI+2000*(theta_ini-2*PI);
float yini=-2500-2000*sin(s/2000);//-cos(dtheta_wheel_diff)*(wheel_diameter_r+wheel_diameter_l)/2*PI/40;
float x_ini=-2000*cos(s/2000);
byte track=0;//0 for cir,1 for line,2 for tilt line,3 for line2
int circuit=0;


float y_c2c_ini=-100;
float c2c_desire=300;
float dacc_dis=459.77;
float acc_dis=287.35;
float acc_set=87;
float line_buffer=100;//+car_length



//filter parameter
const int v_len=5;
const int x_len=5;
float tau_theta = 0.9;
float tau_pos = 0.2;
float tau_ir=0.5;
float tau_vf=0.85;
float tau_a=0.7;
float tau_us_side=0.5;
float tau_st=0.5;
float tau_c2c_dev=0.5;
float tau_br=0.5;
float tau_fan=0.5;
float tau_vn=0.5;

//calculated itself
float psi=atan(middle_to_wheel/wheel_space*2);//rad
float r_prime=sqrt(middle_to_wheel*middle_to_wheel+(wheel_space/2)*(wheel_space/2));
float dtheta_r=1./40*3.1416*wheel_diameter_r/wheel_space;
float dtheta_l=1./40*3.1416*wheel_diameter_l/wheel_space;
float dtheta_wheel_diff=atan((wheel_diameter_r-wheel_diameter_l)*PI/40/wheel_space);
//---------------variable declare----------
Servo steer;
Servo fan;
Servo myservor;
Servo myservol;
float IRf=0,IRb=0;//-2~2
bool IRf_sense=false,IRb_sense=false;
int last_wr_count=0, last_wl_count=0, wr_count=0, wl_count=0, wr_read, wl_read;
float x_ir=0,x_ir_last=0,x_ir_int=0,x=x_ini,x_en=x_ini,x_en_last=x_ini,x_last=x_ini,x_desire=x_ini,x_int=0,
      turn_error=0,turn_error_last=0,turn_error_int=0,
      theta_en=theta_ini,theta_en_last=theta_ini,theta_ir=0,
      theta_last=theta_ini,theta_int=0,theta=theta_ini,
      a=0,a_last=0,v=0,v_last=0,vx=0,vy=0,vf=0,vf_last=0,
      v_error=0,v_error_last=0,v_error_int=0,
      v_en=0,v_need=0,delta_v_us=0,delta_v_us_last=0,v_base=0,y_c2c=y_c2c_ini,
      c2c_error=0,c2c_error_last=0,c2c_error_dev=0,c2c_error_dev_last=0,c2c_error_int=0,
      s_total=0,us_side=1000,us_side_last=1000, s_circuit=0;
float y=yini,y_en=yini,y_en_last=yini,y_last=yini,y_desire=-2499.99;
float stinput=0,stinput_last=0;
//time: all in ms except in PID's I calculation
float current_time=0, vl_last_time=0,
      a_last_time=0,sample_last_time=0,sample_period=50,
      v_en_last_time=0;//milli
float dt=1;//milli
int highlowLabelR=0,highlowLabelL=0;
int brake=0;

int fan_input=fanbase;
float fan_last=fanbase;
bool track1on=false;
float vlist[v_len];
float xlist[x_len];
float x_en_mean=x_ini;
float v_mean=0;
int v_counter=0;
int x_counter=0;
byte en_counter=0;
int stop_check_count=0;
volatile unsigned long usPrevTime[n_us]; //時間
volatile float usDist[n_us];//垂直車輛距離
float usThresh[n_us];//距離值小於多少算是在乎的距離
volatile bool usFlag[n_us];
bool us_side_sense=false;
bool us_side_sense_last=false;
bool leaving=false;
bool after_leaving=false;
byte c2c_on=1;
float s_leave=0;
float y_leave=0;
int brake_last=0;
bool fb=true;
bool us_sense=true;
//-------------sensor setup------------------
bool sensor_setup(){
  steer.attach(steerfoot);
  myservor.attach(brake_r_foot);
  myservol.attach(brake_l_foot);
  //variable setup
  for (int i=0;i<v_len;i++){
    vlist[i]=0;
  }
  for (int i=0;i<x_len;i++){
    xlist[i]=x_ini;
  }
  //
  enforce_steer(0);
  delay(10);
  enforce_brake(br,bl);
  delay(10);
  //track pick
  if(digitalRead(track1)) track1on=true;
  
  //delay(3000);
  //fan.writeMicroseconds(1050);
  //
  MsTimer2::set(1,ticktock);//(ms,interrupt func.)
  MsTimer2::start();
  return true;
}
//-------------encoder update-------------------
int delta_count=0;
bool en_update(){
  sei();//able to be interrupted
  wr_read=analogRead(wr);
  wl_read=analogRead(wl);
  bool en_isupdated=0;
  if(wl_read > encoder_threshold_left_high && highlowLabelL != 1){
        highlowLabelL = 1;
        wl_count += 1;
        stop_check_count=0;
        en_isupdated=1;
        en_counter++;
        s+=wheel_space/2*dtheta_l;
        s_total+=wheel_space/2*dtheta_l;
    }
    else if(wl_read < encoder_threshold_left_low && highlowLabelL != -1){
        highlowLabelL = -1;
        wl_count += 1;
        stop_check_count=0;
        en_isupdated=1;
        en_counter++;
        s+=wheel_space/2*dtheta_l;
        s_total+=wheel_space/2*dtheta_l;
    }
    
    if(wr_read > encoder_threshold_right_high && highlowLabelR != 1){
        highlowLabelR = 1;
        wr_count += 1;
        stop_check_count=0;
        en_isupdated=1;
        en_counter++;
        s+=wheel_space/2*dtheta_r;
        s_total+=wheel_space/2*dtheta_l;
    }
    else if(wr_read < encoder_threshold_right_low && highlowLabelR != -1){
        highlowLabelR = -1;
        wr_count += 1;
        stop_check_count=0;
        en_isupdated=1;
        en_counter++;
        s+=wheel_space/2*dtheta_r;
        s_total+=wheel_space/2*dtheta_l;
    }
    if(en_counter>=en_counter_len){
      delta_count=(wr_count-last_wr_count)-(wl_count-last_wl_count);
      float both_count=min((wr_count-last_wr_count),(wl_count-last_wl_count));
      y_en+=cos(theta+dtheta_wheel_diff*both_count/2)*(wheel_diameter_r+wheel_diameter_l)/2*PI/40*both_count;
      x_en+=-sin(theta+dtheta_wheel_diff*both_count/2)*(wheel_diameter_r+wheel_diameter_l)/2*PI/40*both_count;
      theta_en+=dtheta_wheel_diff*both_count;
      if(delta_count>0){
        y_en+=r_prime*(sin(theta+dtheta_r*delta_count+psi)-sin(theta+psi));//cc+-ss
        x_en+=r_prime*(cos(theta+dtheta_r*delta_count+psi)-cos(theta+psi));//-sc-cs
        theta_en+=dtheta_r*delta_count/2;
        v_en=(both_count+delta_count/2)/(current_time-v_en_last_time)*1000./40*PI*(wheel_diameter_r+wheel_diameter_l)/2;//*2/2
        v_en_last_time=current_time;
        theta_en+=dtheta_r*delta_count/2;
      }
      else if(delta_count<0){
        y_en+=r_prime*(sin(theta+3.1416+dtheta_l*delta_count-psi)-sin(theta-psi+3.1416));//cc+-ss
        x_en+=r_prime*(cos(theta-psi+3.1416+dtheta_l*delta_count)-cos(theta-psi+3.1416));
        theta_en+=dtheta_l*delta_count/2;
        v_en=(both_count-delta_count/2)/(current_time-v_en_last_time)*1000./40*PI*(wheel_diameter_r+wheel_diameter_l)/2;//*2/2
        v_en_last_time=current_time;
        theta_en+=dtheta_l*delta_count/2;
      }
      else{
        v_en=both_count/(current_time-v_en_last_time)*1000./40*3.1416*(wheel_diameter_l+wheel_diameter_r)/2;//*2/2
        v_en_last_time=current_time;
      }
      if(abs(v_en-v_mean)<300){
        v_mean-=vlist[v_counter]/v_len;
        vlist[v_counter]=v_en;
        v_mean+=v_en/v_len;
        v_counter++;
        if(v_counter>=v_len){
          v_counter=0;
        }
      }
      last_wr_count=wr_count;
      last_wl_count=wl_count;
      en_counter=0;
    }
    if(!en_isupdated){stop_check_count++;}
    x_en_mean-=xlist[x_counter]/x_len;
    xlist[x_counter]=x_en;
    x_en_mean+=x_en/x_len;
    x_counter++;
    if(x_counter>=x_len){
      x_counter=0;
    }
  return true;
}
//----------------------IR-------------------
void IR_update(bool printout){
  //IRf
  int openN = 0,add=0;
  for (int i = 0; i < IRsize; i++){
    int r=digitalRead(irf[i]);
    add += -r*(i-2); openN += r;
  }  
  if(openN != 0){
    IRf = (float)add/openN;
    IRf_sense=true;
  }
  else{
    IRf_sense=false;
  }

  //IRb
  openN = 0;add=0;
  for (int i = 0; i < IRsize; i++){
    int r=digitalRead(irb[i]);
    add += -r*(i-2); openN += r;
  } 
  if(openN != 0){
    IRb = (float)add/openN;
    IRb_sense=true;
  }
  else{
    IRb_sense=false;
  }
  if(printout){
    Serial.print("IRf/IRb: ");
    Serial.print(IRf);Serial.print("/");
    Serial.print(IRb);
    }
  theta_ir=atan(-(IRf-IRb)*IR_space/car_length)+theta_ir_bias;
  x_ir=(1-tau_ir)*x_ir_last+tau_ir*((IRb*(car_length-middle_to_wheel-wheel_to_ir)+IRf*(middle_to_wheel+wheel_to_ir))/car_length*IR_space);
  x_ir_last=x_ir;
}
//-------------------us----------------------------
void us_setup(){
  setupUS(0); attachInterrupt(digitalPinToInterrupt(echoPin[0]), risingUS0, RISING);
  setupUS(1); attachInterrupt(digitalPinToInterrupt(echoPin[1]), risingUS1, RISING);
  setupUS(2); attachInterrupt(digitalPinToInterrupt(echoPin[2]), risingUS2, RISING);
  usThresh[2]=us_side_thresh;
}
void setupUS(int n){
  pinMode(trigPin[n], OUTPUT);
  pinMode(echoPin[n], INPUT);
  pingUS(n);
  usDist[n]=3000;//垂直車輛距離
  usFlag[n]=false;
  usPrevTime[n]=0;
  usThresh[n]=us_threshold;
}
void pingUS(int n)
{
  digitalWrite(trigPin[n], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin[n], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin[n], LOW);
}
void risingUS0() { attachInterrupt(digitalPinToInterrupt(echoPin[0]), fallingUS0, FALLING);  usPrevTime[0] = micros(); }
void risingUS1() { attachInterrupt(digitalPinToInterrupt(echoPin[1]), fallingUS1, FALLING);  usPrevTime[1] = micros(); }
void risingUS2() { attachInterrupt(digitalPinToInterrupt(echoPin[2]), fallingUS2, FALLING);  usPrevTime[2] = micros(); }

void fallingUS0() { 
  attachInterrupt(digitalPinToInterrupt(echoPin[0]), risingUS0, RISING);
  usDist[0] = (float)(micros() - usPrevTime[0]) / 2.91 / 2;  
  usFlag[0] = usDist[0] < usThresh[0];
  }
void fallingUS1() { 
  attachInterrupt(digitalPinToInterrupt(echoPin[1]), risingUS1, RISING);
  usDist[1] = (float)(micros() - usPrevTime[1]) / 2.91 / 2;
  usFlag[1] = usDist[1] < usThresh[1];
  }
void fallingUS2() { 
  attachInterrupt(digitalPinToInterrupt(echoPin[2]), risingUS2, RISING);
  usDist[2] = (float)(micros() - usPrevTime[2]) / 2.91 / 2;
  usFlag[2] = usDist[2] < usThresh[2];
  }

void us_update(bool printout){
  pingUS(0);
  delay(10);
  pingUS(1);
  delay(10);
  pingUS(2);
  delay(10);
  if(usFlag[0]&&!usFlag[1]){
    y_c2c=-usDist[0];
    c2c_error=-c2c_desire-y_c2c;
    us_sense=true;
  }
  else if(!usFlag[0]&&usFlag[1]){
    y_c2c=usDist[1];
    c2c_error=c2c_desire-y_c2c;
    us_sense=true;
  }
  else us_sense=false;
  c2c_error_dev=constrain(tau_c2c_dev*(c2c_error-c2c_error_last)/sample_period*1000+(1-tau_c2c_dev)*c2c_error_dev_last,-350,350);
  if(usFlag[2])us_side=us_side_last*(1-tau_us_side)+tau_us_side*usDist[2];
  if(us_side<us_side_threshold)us_side_sense=true;
  else us_side_sense=false;
  if(us_side_sense_last&&!us_side_sense)leaving=true;
  else leaving=false;
  if(printout){
  Serial.print(usDist[0]); Serial.print("/");
  Serial.print(usDist[1]); Serial.print("/");
  Serial.print(usDist[2]); Serial.print("/");
  Serial.print(us_side);Serial.print("/");
  }
}

/*********************ACTS*********************/
// function calls from state data to actuator signals
void enforce_steer(float turn){
  steer.write(constrain(turn,steer_min,steer_max));
}


void enforce_brake(int r,int l){
  
  br=constrain(r,b_min,b_max);
  bl=constrain(l,b_min,b_max);
  myservor.write(br);
  myservol.write(bl);
}

float stop_time=0;
void enforce_fan(int a,bool printout){
  a=constrain(a,fan_min,fan_max);
  a=constrain(a,900,1200);
  fan.writeMicroseconds (a);
  if(printout){Serial.print(a);Serial.print("/");}
  delay(20);
  return;
  
}
/************************************************/
//-----------v cal----------------
float desire_v(float ystart,float yend,float vstart,float adesire,int fanbase_set,int brakebase_start,int brakebase_end){  
  //output desire y dot mm/s
  fanbase = fanbase_set;
  brakebase=brakebase_start+(brakebase_end-brakebase_start)/(yend-ystart)*(s-ystart);
  float desire_v=0;
  if(s>=0)desire_v = sqrt(2*adesire*(s-ystart) + vstart*vstart);
  if(desire_v<min_speed)desire_v=min_speed;
  return desire_v;
}

//----------------complimentory filter-------------
float comp_filter(float high,float* high_last, float low, float tau,float last){
  float comb = (-*high_last+high+last)*tau+low*(1-tau);
  *high_last=high;
  //last=comb;
  return comb;
}
//-----------------PID-----------------------
//with -error as input
int PID(float P,float I,float D,float pdata,float last_data,float *idata){
  int a = (int)(P*pdata+I*(*idata)+D*(pdata-last_data)/sample_period);
  *idata=constrain(*idata+pdata*sample_period/1000,-50,50);//in s
  //Serial.print(*idata);Serial.print("\t");
  //Serial.print(pdata);Serial.print("\t");
  return a;
}
  //---------------PID with derivative avalible---------------
int PID2(float P,float I,float D,float pdata,float ddata,float *idata){
  int a = (int)(P*pdata+I*(*idata)+D*ddata);
  *idata=constrain(*idata+constrain(pdata,-150,150)*sample_period/1000,-200,200);//in s
  return a;
}
int PID3(float P,float I,float D,float pdata,float ddata,float *idata){
  int a = (int)(P*pdata+I*(*idata)+D*ddata);
  *idata=constrain(*idata+constrain(pdata,-150,150)*sample_period/1000,-50,50);//in s
  return a;
}
//---------------PID with D filter--------------
int PID4(float P,float I,float D,float pdata,float last_data,float *idata,float*d_last_data){
  float ddata=*d_last_data*0.1+(pdata-last_data)/sample_period*0.9;
  int a = (int)(P*pdata+I*(*idata)+D*ddata);
  *idata=constrain(*idata+pdata*sample_period/1000,-200,200);//in s
  *d_last_data=ddata;
  return a;
}

//----------------------printout----------------------- 
void printout(char input){
  //filter
  if(input=='f'){
    Serial.print("x y theta,en/ir/filter: ");
    Serial.print(x_en);Serial.print("/");
    Serial.print(x_ir);Serial.print("/");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(theta_en);Serial.print("/");
    Serial.print(theta_ir);Serial.print("/");
    Serial.print(theta,5);
  }
  if(input=='t'){
    Serial.print("x y v a: ");
    Serial.print(x);Serial.print("/");
    Serial.print(x_en,5);Serial.print("/");
    Serial.print(x_en_mean);Serial.print("/");
    Serial.print(x_ir);Serial.print("\t");
    Serial.print(y);Serial.print("\t");
    Serial.print(theta_en);Serial.print("/");
    Serial.print(theta_ir);Serial.print("/");
    Serial.print(theta);Serial.print("\t");
    Serial.print(vy);Serial.print("\t");
    Serial.print(wr_read);Serial.print("/");
    Serial.print(wl_read);Serial.print("\t");
    Serial.print(wr_count);Serial.print("/");
    Serial.print(wl_count);Serial.print("\t");
    Serial.print(delta_count);
    
  }
  if(input=='h'){
    Serial.print("x y v a: ");
    Serial.print(x);Serial.print("/");
    Serial.print(x_en);Serial.print("/");
    Serial.print(x_desire);Serial.print("\t");
    Serial.print(y);Serial.print("\t");
    Serial.print(v);Serial.print("/");
    Serial.print(v_need);Serial.print("/");
    Serial.print(v_base);Serial.print("\t");
    Serial.print(a);Serial.print("\t");
    Serial.print(v_error);Serial.print("\t");
    Serial.print(v_error_int);Serial.print("\t");
    Serial.print(fan_input);Serial.print("\t");
    Serial.print(brakebase);Serial.print("\t");
    Serial.print(fanbase);Serial.print("\t");
    Serial.print(s_total);Serial.print("\t");
    Serial.print(y_c2c);Serial.print("\t");
    Serial.print(track);
    
  }
  if(input=='o'){
    Serial.print("x y v a: ");
    Serial.print(x_desire);Serial.print("/");
    Serial.print(x);Serial.print("/");
    Serial.print(x_en);Serial.print("/");
    Serial.print(x_ir);Serial.print("\t");
    Serial.print(y_desire);Serial.print("/");
    Serial.print(y);Serial.print("\t");
    Serial.print(circuit);Serial.print("/");
    Serial.print(s_total);Serial.print("\t");
    Serial.print(track);Serial.print("\t");
    Serial.print(steerbase);Serial.print("\t");
    Serial.print(turn_error);Serial.print("\t");
    Serial.print(steerbase-PID(0.4,0.05,80,turn_error,turn_error_last,&turn_error_int));
  }
  //steer PID
  if(input=='x'){
    Serial.print(x_desire);Serial.print("/");
    Serial.print(x);Serial.print("/");
    Serial.print(x_en);Serial.print("/");
    Serial.print(x_ir);Serial.print("\t");
    Serial.print(y_desire);Serial.print("/");
    Serial.print(y);Serial.print("\t");
    Serial.print(s_total);Serial.print("\t");
    Serial.print(track);Serial.print("\t");
    Serial.print(v);Serial.print("\t");
    Serial.print(steerbase);Serial.print("\t");
    Serial.print(turn_error);Serial.print("/");
    Serial.print(turn_error_int);Serial.print("/");
    Serial.print((turn_error-turn_error_last)/sample_period);
  }
  //c2c pid
  if(input=='c'){
    Serial.print(x_desire);Serial.print("/");
    Serial.print(x);Serial.print("/");
    Serial.print(x_en);Serial.print("/");
    Serial.print(x_ir);Serial.print("\t");
    Serial.print(y_c2c);Serial.print("/");
    Serial.print(y);Serial.print("\t");
    Serial.print(s_total);Serial.print("\t");
    Serial.print(track);Serial.print("\t");
    Serial.print(v);Serial.print("/");
    Serial.print(v_need);Serial.print("\t");
    Serial.print(fan_input);Serial.print("\t");
    Serial.print(brakebase);Serial.print("/");
    Serial.print(fanbase);Serial.print("/");
    Serial.print(v_base);Serial.print("\t");
    Serial.print(c2c_error);Serial.print("/");
    Serial.print(c2c_error_int);Serial.print("/");
    Serial.print(c2c_error_dev);
  }
}

//-------------------update last data-----------------
void last_data_update(){
  x_last=x;
  y_last=y;
  theta_last=theta;
  v_error_last=v_error;
  v_last=v_mean;
  c2c_error_last=c2c_error;
  sample_period=current_time-sample_last_time;
  sample_last_time=current_time;
  turn_error_last=turn_error;
  us_side_sense_last=us_side_sense;
  us_side_last=us_side;
  stinput_last=stinput;
  c2c_error_dev_last=c2c_error_dev;
  brake_last=brake;
  delta_v_us_last=delta_v_us;
  return;
}
//--------------------v update----------------------
void v_update(){
  if(stop_check_count<300){
    v=v_mean;    
    vx=v_mean*sin(theta);
    vy=v_mean*cos(theta);
  }
  else{
    v=0;
    v_en=0;
    vx=0;
    vy=0;
    v_mean=0;
    for (int i=0;i<v_len;i++){
      vlist[i]=0;
    }
  }
  vf=tau_vf*v+(1-tau_vf)*vf_last;
  a=(vf-vf_last)/(current_time-a_last_time)*1000;
  a=constrain(a*tau_a+(1-tau_a)*a_last,-500,500);
  a_last_time=current_time;
  a_last=a;
  vf_last=vf;
  v_error=v_need-v;
  
}

//-------------------line sense---------------------
void line(float x0,float taupos,float tautheta,float dir){
 if(track!=1)s=0;
 if(IRf_sense && IRb_sense){
    // 1 means all resulted from en, 0 means all resulted from ir.
    if(theta>2*PI-0.25){theta-=2*PI;theta_last-=2*PI;}
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+(1-dir)/2*PI,tautheta,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,dir*x_ir+x0,taupos,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,0,1,y_last);  }
  else{
    if(theta>2*PI-0.25){theta-=2*PI;theta_last-=2*PI;}
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+(1-dir)/2*PI,1,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,dir*x_ir+x0,1,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,0,1,y_last);
  }
  
  steerbase=steer_zero;
  turn_error=dir*(x-x0);
  x_desire=x0;
  y_desire=y;
  track=1;
}
//------------------another line-------------------
void line2(float x0,float taupos,float tautheta,float dir){
 if(track!=3)s=0;
 if(IRf_sense && IRb_sense){
    // 1 means all resulted from en, 0 means all resulted from ir.
    if(theta>2*PI-0.25){theta-=2*PI;theta_last-=2*PI;}
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+(1-dir)/2*PI,tautheta,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,dir*x_ir+x0,taupos,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,0,1,y_last);  
    }
  else{
    if(theta>2*PI-0.25){theta-=2*PI;theta_last-=2*PI;}
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+(1-dir)/2*PI,1,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,dir*x_ir+x0,1,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,0,1,y_last);
  }
  
  steerbase=steer_zero;
  turn_error=dir*(x-x0);
  x_desire=x0;
  y_desire=y;
  track=3;
}
//-------------------circle sense-------------------
void circle(float x0, float y0,float theta0,float radius,float taupos,float tautheta,int steer_delta_pwm,float dir){//counterclockwise=1,clockwise=-1
  if(track!=0)s=0;
  float thetas=dir*s/radius+theta0;
  if(IRf_sense && IRb_sense){
    // 1 means all resulted from en, 0 means all resulted from ir.
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+thetas,tautheta,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,x0+(dir*x_ir+radius)*cos(thetas),taupos,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,y0+(dir*x_ir+radius)*sin(thetas),taupos,y_last);
  }
  else{
    theta=comp_filter(theta_en,&theta_en_last,theta_ir+thetas,1,theta_last);
    x=comp_filter(x_en_mean,&x_en_last,x0+(dir*x_ir+radius)*cos(thetas),1,x_last);//& for sending address 
    y=comp_filter(y_en,&y_en_last,y0+(dir*x_ir+radius)*sin(thetas),1,y_last);
  }
  steerbase=steer_zero-dir*steer_delta_pwm;
  turn_error=dir*(sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0))-radius);
  x_desire=x0+radius*cos(thetas);
  y_desire=y0+radius*sin(thetas);
  track=0;
}
//--------------------tilt line act sense-----------------
void tilt_line(float x0, float y0,float xend, float yend){
  if(track!=2)s=0;
  theta=comp_filter(theta_en,&theta_en_last,theta_ir,1,theta_last);
  x=comp_filter(x_en_mean,&x_en_last,x_ir+x0,1,x_last);//& for sending address 
  y=comp_filter(y_en,&y_en_last,0,1,y_last);
  steerbase=steer_zero;
  turn_error=((x-x0)*(yend-y0)-(y-y0)*(xend-x0))/sqrt((xend-x0)*(xend-x0)+(yend-y0)*(yend-y0));
  y_desire=y;
  x_desire=(y-y0)/(yend-y0)*(xend-x0)+x0;
  track=2;
}
//-------------------s curve sense and act set-------------------
float switch_allow_distance=1000;//delta_y
float shift_distance=325;//delta_x
float turn_radius=700;//r
float turn_theta=25.397*PI/180;//calculated by yourself with calculator
//(delta_y-2rsin(theta))tan(theta)=delta_x-2r(1-cos(theta)) solve theta
int steer_pwm=15;//calculated by yourself with calculator
//table=>x=x-90
float s_sc=turn_radius*turn_theta*2.+sqrt((turn_radius*(1-cos(turn_theta))-(shift_distance-turn_radius*(1-cos(turn_theta))))*(turn_radius*(1-cos(turn_theta))-(shift_distance-turn_radius*(1-cos(turn_theta))))+
                                            ((turn_radius*sin(turn_theta))-(switch_allow_distance-turn_radius*sin(turn_theta)))*((turn_radius*sin(turn_theta))-(switch_allow_distance-turn_radius*sin(turn_theta))));
void s_curve(float x0,float y0,float dir,float outin){//right=1
  float yin=abs(y-y0);
  if(yin<turn_radius*sin(turn_theta)){
    circle(x0+outin*dir*turn_radius,y0,(outin+dir)/2*PI,turn_radius,1,1,steer_pwm,-outin);
  }
  else if(yin<switch_allow_distance-turn_radius*sin(turn_theta)){
    tilt_line(x0+outin*dir*turn_radius*(1-cos(turn_theta)),y0+dir*turn_radius*sin(turn_theta),x0+outin*dir*(shift_distance-turn_radius*(1-cos(turn_theta))),y0+dir*(switch_allow_distance-turn_radius*sin(turn_theta)));
  }
  else if(yin<switch_allow_distance){
    circle(x0+outin*dir*(shift_distance-turn_radius),y0+dir*switch_allow_distance,-outin*turn_theta+(2-dir-outin)/2*PI,turn_radius,1,1,steer_pwm,outin);
  }
  else{
    line(x0+shift_distance,tau_pos,tau_theta,dir);
  }
}
//-------------------v design------------------
float v_design(float vdr,float vin,float vout,float track_len,int fanbase_set,int bbase_setst,int bbase_setend){
  //desire_v(y1,      y2,   v1,    a,    f,  b1,   b2)
  float s1=abs(vdr*vdr-vin*vin)/2/acc_set;
  float s2=abs(vdr*vdr-vout*vout)/2/acc_set;
  if(s<0)return vin;
  else if(s<s1){
    if(vin<vdr)return desire_v(0,s1,vin,acc_set,1115,90,100);
    else if(vin>vdr)return desire_v(0,s1,vin,-acc_set,1080,100,120);
    else return desire_v(0,s1,vdr,0,fanbase_set,bbase_setst,bbase_setend);
  }
  else if(s<track_len-s2){
    return desire_v(s1,track_len-s2,vdr,0,fanbase_set,bbase_setst,bbase_setend);
  }
  else if(s<track_len){
    if(vout>vdr)return desire_v(track_len-s2,track_len,vdr,acc_set,1115,90,100);
    else if(vout<vdr)return desire_v(track_len-s2,track_len,vdr,-acc_set,1080,100,120);
    else return desire_v(track_len-s2,track_len,vdr,0,fanbase_set,bbase_setst,bbase_setend);
  }
  else return vout;
}
//------------------x map-----------------------
int fanbase_reg=fanbase0;
int bbase_reg=100;
float total_length=22566.37+2*(s_sc*2-switch_allow_distance);
float fcv=500;//free_circuit_velocity
void x_map(){
  //desire_v(y1,   y2,   v1,    a,    f,  b1,   b2)
  //v_design(vdr,vin,vout,track_len,fanbase_set,bbase_set)
  if(circuit==2){
    if(y>=2500){
      circle(0,2500,0,2000,0,0,5,1);
      v_base=desire_v(0,6284,350,0,fanbase_reg,bbase_reg,bbase_reg);
      c2c_on=1;
      fb=false;
    }
    else if(y<=-2500){
      circle(0,-2500,PI,2000,0,0,5,1);
      v_base=desire_v(0,6284,350,0,fanbase_reg,bbase_reg,bbase_reg);
      c2c_on=1; 
      fb=true; 
    }
    else if(x>0){
      if(y<-2500+car_length+line_buffer){
        if(track==0)circuit+=1;
        line(2000,tau_pos,tau_theta,1);
        v_base=v_design(350,350,350,50,fanbase_reg,bbase_reg,bbase_reg);
        after_leaving=false;
        c2c_on=0;
        fb=true;
      }
      else if(y<-2500+car_length+line_buffer+switch_allow_distance){
        s_curve(2000,-2500+car_length+line_buffer,1,-1);
        v_base=desire_v(0,2000,350,0,fanbase_reg,100,100);//yend didn't calculate
        after_leaving=false;
        c2c_on=0;
      }
      else if(!leaving&&!after_leaving){
        line(2000-shift_distance,tau_pos,tau_theta,1);
        v_base=v_design(450,350,450,3000,fanbase_reg,100,100);
        c2c_on=0;
      }
      else if(leaving&&!after_leaving){
        leaving=false;
        after_leaving=true;
        y_leave=y;
      }
      else if(y<y_leave+dacc_dis){
        line2(2000-shift_distance,tau_pos,tau_theta,1);
        v_base=v_design(350,450,350,dacc_dis,fanbase_reg-20,100,100);
        c2c_on=0;
      }
      else if(y<y_leave+dacc_dis+switch_allow_distance){
        s_curve(2000-shift_distance,y_leave+dacc_dis,1,1);
        v_base=desire_v(0,2000,350,0,fanbase_reg,100,100);//yend didn't calculate
        c2c_on=0;
      }
      else if(y<2500){
        line(2000,tau_pos,tau_theta,1);
        v_base=v_design(350,350,350,2000,fanbase_reg-15,bbase_reg,bbase_reg);
        c2c_on=1;
        fb=false;
      }
      else{
        line(2000,tau_pos,tau_theta,1);
        v_base=v_design(350,350,350,2000,fanbase_reg,bbase_reg,bbase_reg);
        c2c_on=1;
        fb=false;
      }
    }
    else if(x<0){
      if(y>2500-car_length-line_buffer){
        line(-2000,tau_pos,tau_theta,-1);
        v_base=v_design(350,350,300,50,fanbase_reg,bbase_reg,bbase_reg);
        after_leaving=false;
        c2c_on=0;
        fb=false;
      }
      else if(y>2500-car_length-line_buffer-switch_allow_distance){
        s_curve(-2000,2500-car_length-line_buffer,-1,-1);
        v_base=desire_v(0,2000,300,0,fanbase_reg,100,100);//yend didn't calculate
        after_leaving=false;
        c2c_on=0;
      }
      else if(!leaving&&!after_leaving){
        line(-2000+shift_distance,tau_pos,tau_theta,-1);
        v_base=v_design(200,300,200,3000,fanbase_reg-15,100,100);
        c2c_on=0;
      }
      else if(leaving&&!after_leaving){
        leaving=false;
        after_leaving=true;
        y_leave=y;
        c2c_on=0;
      }
      else if(y>y_leave-acc_dis){
        line2(-2000+shift_distance,tau_pos,tau_theta,-1);
        v_base=v_design(300,200,300,acc_dis,fanbase_reg+25,100,100);
        c2c_on=0;
        theta_ir_bias=-0.025;
      }
      else if(y>y_leave-acc_dis-switch_allow_distance){
        s_curve(-2000+shift_distance,y_leave-acc_dis,-1,1);
        v_base=desire_v(0,2000,300,0,fanbase_reg+15,100,100);
        c2c_on=0;
        theta_ir_bias=-0.01;
      }
      else if(y>-2500){
        line(-2000,tau_pos,tau_theta,-1);
        v_base=v_design(350,300,350,2000,fanbase_reg+15,bbase_reg,bbase_reg);
        c2c_on=1;
        fb=true;
      }
      else{
        line(-2000,tau_pos,tau_theta,-1);
        v_base=v_design(350,350,350,2000,fanbase_reg,bbase_reg,bbase_reg);
        c2c_on=1;
        fb=true;
      }
    }
  }
  //
  else{
    if(y>=2500){
      circle(0,2500,0,2000,0,0,5,1);
      v_base=desire_v(0,6284,fcv,0,fanbase0,bbase_reg,bbase_reg);
      c2c_on=1;
      fb=true;
    }
    else if(y<=-2500){
      circle(0,-2500,PI,2000,0,0,5,1);
      v_base=desire_v(0,6284,fcv,0,fanbase0+15,bbase_reg,bbase_reg);
      c2c_on=1;  
      fb=true;
    }
    else if(x>0){
      if(track!=3)circuit+=1;
      line2(2000,tau_pos,tau_theta,1);
      v_base=v_design(fcv,fcv,fcv,5000,fanbase0,bbase_reg,bbase_reg);
      c2c_on=1;
      fb=true;
    }
    else{
      line(-2000,tau_pos,tau_theta,-1);
      v_base=v_design(fcv,fcv,fcv,5000,fanbase0,bbase_reg,bbase_reg);
      c2c_on=1;
      fb=true;
    }
  }
  if(circuit>4)fanbase=900;
  if(circuit==3)fanbase0=fanbase00+10;
}
//==================================================
//==================arduino setup==================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  fan.attach(fanfoot);
  fan.writeMicroseconds(950);
  delay(3000);
  fan.writeMicroseconds(fanbase);  
  sensor_setup();
  us_setup();
  //sensor_setup();//don't know why it ok for only twice
}
//==================-keep running=================
//      日一日
//      | ^ |
//     /|| ||\
//     U w w U
long ii=0;
void loop() {
  // put your main code here, to run repeatedly:
  IR_update();
  us_update(false);
  delta_v_us=tau_vn*(PID2(1.6,0,0.5,c2c_error,c2c_error_dev,&c2c_error_int))+(1-tau_vn)*delta_v_us_last;
  if(us_sense||c2c_on<0.5){
    v_need=v_base+delta_v_us*float(c2c_on); 
  }
  else{
    if(fb)v_need=700;
    else v_need=200;
  }
  v_update();
  x_map();
  fan_input=tau_fan*(fanbase0+PID3(2,3,0.2,v_error,-a,&v_error_int))+(1-tau_fan)*fan_last;
  fan_input=constrain(fan_input,fan_min,fan_max);
  enforce_fan(fan_input,false);
  enforce_brake(brake,180-brake);
  stinput=(1-tau_st)*stinput_last+tau_st*(steerbase-PID(0.4,0.05,60,turn_error,turn_error_last,&turn_error_int));
  enforce_steer(stinput);
  Serial.print(current_time);Serial.print(": ");
  printout('c');
  Serial.print("\n");
  last_data_update();
}
void ticktock(){
  current_time+=dt;
  en_update();
}
