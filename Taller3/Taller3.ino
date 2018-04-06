#include <Encoder.h>

#define SIMULACION 1

/* Definiciones de pines */
#define PIN_MOT_A 7
#define PIN_MOT_B 6
#define PIN_ENC_A 20
#define PIN_ENC_B 21

#if SIMULACION
#define analogWrite analogWriteSim
#else
Encoder enc(PIN_ENC_A, PIN_ENC_B);
#endif

/* Variables globales */
float prev_pos = 0;  // pos anterior del encoder
float tiempo_anterior_control_loop;

float ITerm = 0;
double pid_input, pid_output, pid_setpoint, pid_last_input;

/*constantes globales pid */
#define PWM_MIN 50
#define PWM_MAX 255
#define PID_P  20
#define PID_I  20
#define PID_D  30
#define CICLO_CONTROL_TIEMPO 10 // milisegundos
double velocidadP = 0; // variables para vel promedio
int cuentas = 0;

void setup()
{ 
  // seteo serial
  Serial.begin(115200);
   
  pid_setpoint = 5;
  pid_last_input = 0;
  tiempo_anterior_control_loop = millis();
  


 // inicializo los pines del motor
  pinMode(PIN_MOT_A, OUTPUT);      // inicializo el pin A del motor como una salida
  pinMode(PIN_MOT_B, OUTPUT);      // inicializo el pin B del motor como una salida
  
  // seteo en 0 la velocidad del motor
  analogWrite(PIN_MOT_A,0);        
  analogWrite(PIN_MOT_B,0);
  
  Serial.print("setpoint: "); Serial.print(pid_setpoint); 
  Serial.print(" p: "); Serial.print(PID_P);
  Serial.print(" i: "); Serial.print(PID_I);
  Serial.print(" d: "); Serial.println(PID_D);
}

/****************************** CICLO PRINCIPAL ***************************/

void loop()
{
  #if SIMULACION
  actualizar_simulacion();
  #endif

  ciclo_control();  

// Serial.print("vel: "); Serial.print(velocidad); Serial.print(" pwm: "); Serial.print(pwm); Serial.print(" t: "); Serial.print(tiempo_control);Serial.print(" delta_t: "); Serial.println(delta_t);
 
}

/***************************** CICLO DE CONTROL **********************/

void ciclo_control()
{
  float tiempo_control = millis();
  int delta_t = tiempo_control  - tiempo_anterior_control_loop;
  if (delta_t >= CICLO_CONTROL_TIEMPO)
  {
    tiempo_anterior_control_loop = tiempo_control;

    float velocidad = calcular_velocidad(delta_t);

    /************* AGREGUE EL CODIGO AQUI **************/

    int pwm = 100; // escribir en esta variable el pwm que se manda al motor (como valor positivo/negativo) en 55 se  mueve
    set_motor_pwm(pwm);
//Serial.print("El tiempo corre: "),Serial.println(tiempo_control),Serial.print("delta t: "),Serial.println(delta_t);

  Serial.print("vel: "); Serial.print(velocidad,10); Serial.print(" pwm: "); Serial.print(pwm); Serial.print(" t: "); Serial.print(tiempo_control,10);Serial.print(" delta_t: "); Serial.println(delta_t);
 
  }  

}


/************************* FUNCIONES AUXILIARES ***********************************/

float calcular_velocidad(double delta_t)
{
  /************* AGREGUE EL CODIGO AQUI **************/
  
  float vel = (encoder_position()-prev_pos)*360/480/delta_t; // creo que delta_t esta en milisegundos
  prev_pos = encoder_position();
 
float velocidadP = vel+velocidadP;
 int cuentas = cuentas + 1;
 float velP= velocidadP/cuentas;
// delay(1000);
  //Serial.print("La velocidad promedio es: "),Serial.println(velP), Serial.print("La velocidad medida es "), Serial.println(vel); 
  return vel; // modificame
}

void set_motor_pwm(int pwm)
{
  /************* AGREGUE EL CODIGO AQUI **************/
  if(pwm >=0 && abs(pwm)<= 255){
    analogWrite(PIN_MOT_B,abs(pwm));
    analogWrite(PIN_MOT_A,0);
  }
  if(pwm <0 && abs(pwm)<= 255){
    analogWrite(PIN_MOT_A,abs(pwm));
    analogWrite(PIN_MOT_B,0);
  }
}

long encoder_position(void)
{
  #if SIMULACION
  return enc_read_sim();
  #else
  return enc.read();
  #endif  
}

/********************************** funciones de simulacion ****************************/

#define SIM_VEL_MIN 0.1 // velocidad minima en la que el motor empieza a moverse
#define SIM_MAX_PWM_QUIETO 50  // pwm maximo para el cual no se mueve

/* Variables globales */
int sim_pwmA = 0;    // asociado al pin motA
int sim_pwmB = 0;    // asociado al pin motB
float sim_vel = 0;   // vel actual
float sim_pos = 0;   // pos actual
int sim_pwm = 0;
float sim_t = 0;
float sim_t_prev = 0;

void actualizar_simulacion()
{
  sim_t = micros() / 1000.00;
  
  sim_pwm = -sim_pwmA + sim_pwmB;
  unsigned long dtsim= sim_t - sim_t_prev;
  if(dtsim == 0) return;
  sim_t_prev = sim_t;
  float alpha1=0.01823;       
  float alpha2=-0.2583;      
  long tau=30*dtsim;
  float NI = 0;
  if(fabs(sim_vel) > SIM_VEL_MIN){  //velocidad dist de 0    
    NI=(sim_vel - alpha2*copysign(1, sim_vel))/tau;
  }
  else{ 
    if(sim_pwm >= SIM_MAX_PWM_QUIETO) NI = alpha1 * SIM_MAX_PWM_QUIETO/tau;
    else if (sim_pwm <= -SIM_MAX_PWM_QUIETO) NI = -alpha1 * SIM_MAX_PWM_QUIETO/tau;
    else{
      sim_vel = 0; 
      NI = alpha1*sim_pwm/tau;
    }
  }       
  //Euler
  float F = -NI+(alpha1*sim_pwm)/tau;
  sim_vel = sim_vel + float(F * dtsim);
  sim_pos = (sim_pos+sim_vel*dtsim);
}

long enc_read_sim()
{
  return sim_pos;
}

void analogWriteSim(int pin, int pwm)
{
  if(pin == PIN_MOT_A) sim_pwmA = pwm;
  if(pin == PIN_MOT_B) sim_pwmB = pwm;  
}


