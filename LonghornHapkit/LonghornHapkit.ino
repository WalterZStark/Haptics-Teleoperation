
//--------------------------------------------------------------------------
// Ann Majewicz Fey, University of Texas at Austin
// Last Modified: 08.27.21
// Code to test basic functionaility of the Longhorn Hapkit (w/ encoder)
//--------------------------------------------------------------------------

// INCLUDES
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <TimerOne.h>  // This library manages the timing of the haptic loop 
#include <Encoder.h>   // This library manages the encoder read.


double stall_Torque = 0.0167; // N*m 

// Pin Declarations
const int PWMoutp = 4;
const int PWMoutn = 5;
const int PWMspeed = 11;

const int encoder0PinA = 2;
const int encoder0PinB = 3;

Encoder encoder(encoder0PinA,encoder0PinB);

double encoderResolution = 48;
double pos = 0; 
double lastPos = 0; 
double lastVel = 0; 
double lastXh = 0; 
double lastLastXh = 0;
double lastVh = 0;
double lastLastVh = 0;


// Kinematics variables
double xh = 0;           // position of the handle [m]
double vh = 0;
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// *******************************************
// UNCOMMENT THESE AND INCLUDE CORRECT NUMBERS
// *******************************************
double rh = 0.089414;   //[m] 
double rp = 0.005;  //[m] 
double rs = 0.075;  //[m] 
// *******************************************

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false; 
boolean timeoutOccured = false; 


// New Variables added by team

double max_force = 2.45; // [N] based on cable slip

// Virtual Wall Variables
double x_wall = 0.005; // Value in meters
double K_wall = 120; // Value in N*m


// Linear Damping Variables
double B = 0.85; // N*s/m, linear damping coefficient


// Dyanmic Friction
double C = 0.5; // N


// Static Friction
double D = 2; // N
double deltaV = 1; // m/s


// Choose mode
bool new_mode = true;
char temp_input = 'w';
char mode = 'w';
// SET SERIAL INPUT SETTING TO NO LINE ENDING OR IT WILL NOT WORK
// Wall : w
// Linear Damping: l
// Nonlinear Friction: n 
// Hard Surface: h
// Bump and Valley: b
// Texture: t



// Hard Surface
bool still_touching = true;
int force_delta = 0;
float pi = 3.1415926;
int oscilation_microsecond_start = 0; // [microseconds] used to track oscilation phase

float oscilation_magnitude_scale = 0.1; // [Ns/m]
float oscilation_decay_base = 2.71828; // e
float oscilation_frequency = 1; // [Hz]


// Bump and Valley
float bump_force_scale = 0.5;
float bump_center = -0.05; // [m]
float bump_width =0.01; // [m]
float valley_force_scale = 0.5;
float valley_center = 0.05; // [m]
float valley_width =0.01; // [m]



//Texture
float texture_speed_threshold = 0.01; // [m/s]
float sawtooth_magnitude = 0.2; // [N] Keep lower than stiction of mechanism
int sawtooth_scale = 1000; // [s -> ms]
int sawtooth_width = 10; // number of time units -> distance units per sawtooth









//--------------------------------------------------------------------------
// Initialize
//--------------------------------------------------------------------------
void setup()
{
  // Set Up Serial
  Serial.begin(115200);

  // Output Pins
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(PWMspeed, OUTPUT);

  // Haptic Loop Timer Initalization
  Timer1.initialize(); 
  long period = 1000; // [us]  10000 [us] - 100 Hz 
  Timer1.attachInterrupt(hapticLoop,period); 

  // Init Position and Velocity
  lastPos = encoder.read();
  lastVel = 0;
  

  // Initalize motor direction and set to 0 (no spin)
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
  analogWrite(PWMspeed, 0);
}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------

void loop()
{
    if(timeoutOccured)
  {
    Serial.println("timeout occured");
  }
}

// --------------------------
// Haptic Loop
// --------------------------
void hapticLoop()
{
  
  // See if flag is out (couldn't finish before another call) 
  if(hapticLoopFlagOut)
  {
    timeoutOccured = true;
  }

  //*************************************************************
  //*** Section 1. Compute position and velocity using encoder (DO NOT CHANGE!!) ***  
  //*************************************************************
  pos = encoder.read();
  double vel = (.80)*lastVel + (.20)*(pos - lastPos)/(.01);


  //*************************************************************
  //*** Section 2. Compute handle position in meters ************
  //*************************************************************
  // ADD YOUR CODE HERE

  // SOLUTION:
  // Define kinematic parameters you may need
  //int CPR = 48; // Counts per revolution


  // Step 2.1: print updatedPos via serial monitor
  //*************************************************************
  // Need the count of number of rotations
  double updatedPos = (pos)/encoderResolution;

  // Step 2.2: Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  //*************************************************************
  double ts = rp*updatedPos*360/(rs);
  //double ts = -.0107*updatedPos + 4.9513; // NOTE - THESE NUMBERS MIGHT NOT BE CORRECT! USE KINEMATICS TO FIGRUE IT OUT!
  // Step 2.3: Compute the position of the handle based on ts
  //*************************************************************
  xh = rh*(ts*3.14159/180);       // Again, these numbers may not be correct. You need to determine these relationships. 

  // Step 2.4: print xh via serial monitor
  //*************************************************************

  // Serial.print(xh,3);
     
  // Step 2.5: compute handle velocity
  //*************************************************************
  vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
  lastXh = xh;
  lastLastVh = lastVh;
  lastVh = vh;
        
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  /*
  // Init force 
  double force = 0.5; // Force in newtons
  double K = 125;   // spring stiffness
  Tp = (rh*rp)/rs * force; // output torque
  //Serial.println(Tp,5);
  force = -K*xh;
        
  */

  // This is just a simple example of a haptic wall that only uses encoder position.
  // You will need to add the rest of the following cases. You will want to enable some way to select each case. 
  // Options for this are #DEFINE statements, swtich case statements (i.e., like a key press in serial monitor), or 
  // some other method. 
    
    // Select mode
    if (Serial.available() > 0) {
    // read the incoming byte:
    char temp_input = Serial.read();    
    mode = char(temp_input);
    new_mode = true;
    }
    else{
      new_mode = false;
    }

    // Switch between modes - occasionally reads 1 mode behind entered
    switch (mode) {
  case 'w':
      // Virtual Wall (w)
      //*************************************************************    
      if(new_mode){
        Serial.println("Wall");
      }
          
      if (xh > x_wall) // IF position < x_wall
      {
        // Added min and maximum to prevent buzzing and slipping on cable
        // Chose K_wall to be slightly less than K = 125 becasue K=125 is the maximum K value we found that preforms well
        force = constrain(-K_wall*(xh-x_wall),-1*max_force, -1); // THEN Force = - k_wall  x_wall
      }
      else // ELSE
      {
        force = 0; // THEN Force = 0
      }
      break;
      
  case 'l':
    // Linear Damping (l)
    //*************************************************************
    if(new_mode){
        Serial.println("Linear Damping");
      }
    force = -vh*B; // Implement Linear Damping Equation
    break;

  case 'n':
    // Nonlinear Friction (n)
    //*************************************************************
    if(new_mode){
        Serial.println("Nonlinear Friction");
      }
    
    
  // double Fa = a * m;
    if(abs(vh)<deltaV) // IF velocity is small enough for static friction
    {
      force = -sign(vh)*D; // Find the force of hand from acceleration
    }
  
    else // ELSE implement vicous damping and dynamic friction
    {
      force = -C*sign(vh) - B*vh;
    }
  
    //Serial.println(abs(vh));

    break;

    
  case 'h':
    // A Hard Surface (h)
    //*************************************************************
    if(new_mode){
      Serial.println("Hard Surface");
    }
    if (xh > x_wall) // IF position < x_wall
    {
      
      if (still_touching) // Check to see if continuing previous sinusoid
      {
        // Equation equivalent to:
        // (OMS * Vh) * (ODB^(-(t-t0)) * sin(2*pi*OF*(t-t0)))

        force_delta = oscilation_magnitude_scale * vh * // Initial Magnitude
                      pow(oscilation_decay_base, -(micros()-oscilation_microsecond_start)*pow(10,-6))* // decay component
                      sin(2*pi*oscilation_frequency*(micros()-oscilation_microsecond_start)*pow(10,-6)); // sinusoidal component
      }
      else
      {
        still_touching = true;
        oscilation_microsecond_start = micros();
        force_delta = 0;
      }

      // Added min and maximum to prevent buzzing and slipping on cable
      // Chose K_wall to be slightly less than K = 125 becasue K=125 is the maximum K value we found that preforms well
      force = constrain(-K_wall*(xh-x_wall) + force_delta,-1*max_force, -1); // THEN Force = - k_wall  x_wall
    }
    else // ELSE
    {
      force = 0; // For sinusoid, force starts at 0
      still_touching = false;
    }

    break;


  case 'b':
    // Bump and Valley (b)
    //*************************************************************
    if(new_mode){
        Serial.println("Bump and Valley");
      }

    // Render Bump
    if(abs(bump_center - xh) < bump_width)
    {
      // May need to put a negative
      force = max_force * bump_force_scale * cos(pi* (bump_center - xh)/bump_width); // Sinusoidally varies force based on proximity to center of bump
    }
    
    // Render Valley
    if(abs(valley_center - xh) < valley_width)
    {
      // May need to remove negative
      force = -1 * max_force * valley_force_scale * cos(pi* (valley_center - xh)/valley_width); // Sinusoidally varies force based on proximity to center of bump
    }
    break;


  case 't':
    // Texture (t)
    //*************************************************************
    if(new_mode){
        Serial.println("Texture");
    }
    
    if(abs(vh) > texture_speed_threshold)
    {
      force = sawtooth_magnitude * (int(vh*sawtooth_scale) % sawtooth_width) / sawtooth_width; // multiply velocity by factor with modulus to produce sawtooth effect based on speed
    }
  else
  {
    force = 0; // Set force to 0 when not moving along surface
  }
    break;
      // CHALLENGE POINTS: Try simulating a paddle ball! Hint you need to keep track of the virtual balls dynamics and 
      // compute interaction forces relative to the changing ball position.  
  //*************************************************************

  default:
  if(new_mode){
        Serial.println("No Mode. Options:\nw -> Wall\nl -> Linear Damping\nn -> Nonlinear Friction\nh -> Hard Surface\nb -> Bump and Valley\nt -> Texture\n[CASE SENSITIVE, SELECT NO LINE ENDING]");

      }
  

  }
      //*************************************************************
      //*** Section 4. Force output (do not change) *****************
      //*************************************************************
      // Serial.println("Section 4");
        // Determine correct direction 
        //*************************************************************
        if(force < 0)
        {
        digitalWrite(PWMoutp, HIGH);
        digitalWrite(PWMoutn, LOW);
        } else 
        {
         digitalWrite(PWMoutp, LOW);
        digitalWrite(PWMoutn, HIGH);
        } 
    
        // Convert force to torque, limit torque to motor, and write out
        //*************************************************************
         Tp = (rh*rp)/rs * force; // ans: force*(rh*rp)/rs; //torque = ? See slides for relationship
          
         // Add some code here to limit Tp based on the stall torque. 
         if(abs(Tp) > stall_Torque)
         {
          Tp = stall_Torque;
         }

        // Write out the motor speed.
        //*************************************************************    
        analogWrite(PWMspeed, (abs(Tp)/stall_Torque)*255); // This ensures we aren't writing 

    
  // Update variables 
  lastVel = vel;
  lastPos = pos; 

}

// Function for returning the sign of a variable


int sign(double value) {
  if (value > 0) {
    return 1;  // Positive
  } else if (value < 0) {
    return -1; // Negative
  } else {
    return 0;  // Zero
  }


}

