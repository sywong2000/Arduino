// small gear 16 teeth, large gear 35 teeth, gear ratio is 16/35
// 4 degrees needs 1 full large gear turn = 35/16 small gear turn = 35/16 * 1600 microsteps (i.e. 3500 microsteps)
// 4*PI/180 = 3500 steps
// 1 degree = PI /180 = 875 steps

const double rad_per_step = PI / (180.0 * 875.0); //radian per microstep (1600 steps per cycle)
const double step_per_rad = (180.0 * 875.0) / PI; //radian per microstep (1600 steps per cycle)
const double speed_factor = 1.0;

double gaz;
double gel;
double gphi;
double gha;
double gdec;
double gazd;
double gazdd;
double geld;
double geldd;
double gpa;
double gpad;
double gpadd;

unsigned long lastElMicros;
unsigned long lastAzMicros;
unsigned long lastMicros;
unsigned long currentMicros;
unsigned long dElMicros;
unsigned long dAzMicros;
unsigned long dMicros;
double dElDelay;
double dAzDelay;
const unsigned long period = 100;  //the value is a number of Microseconds
boolean bRecal = true;

const int EnablePin = 8;
const int stepperPin1 = 2;
const int stepperPin2 = 3;
const int dirPin1 = 5;
const int dirPin2 = 6;
//const int msx1pin = 4;
//const int msx2pin = 5;
//const int msy1pin = 8;
//const int msy2pin = 9;

// sidereal tracking rate 
const double u = 0.00007272;
const double usq = u * u;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.print("Step Per Rad = ");
  Serial.print(step_per_rad);
  Serial.println();

  // init the stepper
  pinMode(dirPin1, OUTPUT);
  pinMode(stepperPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepperPin2, OUTPUT);
  
  pinMode(EnablePin, OUTPUT);

  
  // assume 90 due east, 45 degree up
  gaz = 90 * PI / 180;
  gel = 45 * PI / 180;
  // hong kong
  gphi = 22.28552 * PI / 180;
  AzElConvert(gaz, gel, gphi, &gha, &gdec, &gazd, &gazdd, &geld, &geldd, &gpa, &gpad, &gpadd);
  Serial.print("geld = ");
  Serial.print(geld,10);
  Serial.print("\t");
  Serial.print("gazd = ");
  Serial.print(gazd,10);
  Serial.println();

  dElDelay = (rad_per_step/geld) * 1000000.0/speed_factor;
  dAzDelay = (rad_per_step/gazd) * 1000000.0/speed_factor;
  
  Serial.print("dElDelay = ");
  Serial.print(dElDelay);
  Serial.print("\t");
  Serial.print("dAzDelay = ");
  Serial.print(dAzDelay);
  Serial.println();

  digitalWrite(dirPin1,dElDelay>0);
  digitalWrite(dirPin2,dAzDelay>0);
  digitalWrite(stepperPin1, LOW);
  digitalWrite(stepperPin2, LOW);
  digitalWrite(EnablePin, LOW);

  lastElMicros = micros();
  lastAzMicros = micros();
  lastMicros = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(".");
  currentMicros = micros();

  dElMicros = currentMicros - lastElMicros;
  dAzMicros = currentMicros - lastAzMicros;
  dMicros = currentMicros - lastMicros;
  
  if (dElMicros >= abs(dElDelay-200))
  {
    digitalWrite(dirPin1,dElDelay>0);
    digitalWrite(stepperPin1, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepperPin1, LOW);


//    double rpm =  60000000.0 / dElDelay * 1600;
//    Serial.print("El RPM = ");
//    Serial.print(rpm,10);
    
    Serial.print("Step El. dElDelay = ");
    Serial.print(dElDelay,10);
    Serial.println();
    lastElMicros = currentMicros;
  }
  
  // minus 200 as the delay buffer
  
  if (dAzMicros >= abs(dAzDelay)-200)
  {
    
    digitalWrite(dirPin1,dAzDelay>0);
    digitalWrite(stepperPin2, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepperPin2, LOW);

    Serial.print("Step Az. dAzDelay = ");
    Serial.print(dAzDelay,10);
    Serial.println();
   
    lastAzMicros = currentMicros;
  }

  if (dMicros>200000)
  {
    gaz +=  gazd * ((double)dMicros / 1000000.0);
    gel +=  geld * ((double)dMicros / 1000000.0);
    
    AzElConvert(gaz, gel, gphi, &gha, &gdec, &gazd, &gazdd, &geld, &geldd, &gpa, &gpad, &gpadd);
    dElDelay = (rad_per_step/geld) * 1000000.0/speed_factor;
    dAzDelay = (rad_per_step/gazd) * 1000000.0/speed_factor;
    
    Serial.print("az =");
    Serial.print(gaz * (180.0/PI), 10);
    Serial.print(" deg.");
    Serial.println();
    Serial.print("alt=");
    Serial.print(gel * (180.0/PI), 10);
    Serial.print(" deg.");
    Serial.println();
    
    Serial.print("az speed =");
    Serial.print(gazd * (180.0/PI), 10);
    Serial.println();
    Serial.print("alt speed=");
    Serial.print(geld * (180.0/PI), 10);
    Serial.println();
    
    lastMicros = micros();
  }
}


void AzElConvert(double az, double el, double phi, double *ha, double *dec, double *azd, double *azdd, double *eld, double *eldd, double *pa, double *pad, double *padd)
{

  // http://www.ing.iac.es/~docs/tcs/software/TCS_PAPER_RL.pdf
  // https://github.com/Starlink/pal

  /*
                 Arguments:
        ha = double (Given)
           Hour angle (radians)
        dec = double (Given)
           Declination (radians)
        phi = double (Given)
           Observatory latitude (radians)
        az = double * (Returned)
           Azimuth (radians)
        azd = double * (Returned)
           Azimuth velocity (radians per radian of HA)
        azdd = double * (Returned)
           Azimuth acceleration (radians per radian of HA squared)
        el = double * (Returned)
           Elevation (radians)
        eld = double * (Returned)
           Elevation velocity (radians per radian of HA)
        eldd = double * (Returned)
           Elevation acceleration (radians per radian of HA squared)
        pa = double * (Returned)
           Parallactic angle (radians)
        pad = double * (Returned)
           Parallactic angle velocity (radians per radian of HA)
        padd = double * (Returned)
           Parallactic angle acceleration (radians per radian of HA squared)
  */

  double sa = sin(az);
  double ca = cos(az);
  double se = sin(el);
  double ce = cos(el);
  double sp = sin(phi);
  double cp = cos(phi);

  double sz = sin(HALF_PI - el);
  double cz = cos(HALF_PI - el);


  double x = -ca * ce * sp + se * cp;
  double y = -sa * ce;
  double z = ca * ce * cp + se * sp;

  /*  To HA,Dec */
  double r = sqrt(x * x + y * y);
  double h, d;
  if (r == 0.) {
    h = 0.;
  } else {
    h = atan2(y, x);
  }
  d = atan2(z, r);

  double cd = cos(d);
  double sd = sin(d);
  double ch = cos(h);
  double sh = sin(h);

  double p = asin(-sa * cp / cd);
  double cpa = cos(p);
  double spa = sin(p);

  //double ed = -sa * cp * u;
  double ed = sa * cp * u;
  double edd = cp * (cd * ch - sa * sa * cp * cz) / sz * usq;
  double ad = (sp * sz - cp * cz * ca) / sz * u;
  double add = cd * cp * (cpa * cz * sa - spa * ca) / (sz * sz) * usq;

  double pd = cp * ca / sz;
  double pdd = sa * cp * (cpa * cd - ca * cz * cp) / (sz * sz);


  *ha = h;
  *dec = d;
  *eld = ed;
  *eldd = edd;
  *azd = ad;
  *azdd = add;
  *pa = p;
  *pad = pd;
  *padd = pdd;


}

void Dh2e ( double az, double el, double phi, double *ha, double *dec) {

  /*
     Arguments:
        az = double (Given)
           Azimuth (radians)
        el = double (Given)
           Elevation (radians)
        phi = double (Given)
           Observatory latitude (radians)
        ha = double * (Returned)
           Hour angle (radians)
        dec = double * (Returned)
           Declination (radians)
  */
  double sa;
  double ca;
  double se;
  double ce;
  double sp;
  double cp;

  double x;
  double y;
  double z;
  double r;

  /*  Useful trig functions */
  sa = sin(az);
  ca = cos(az);
  se = sin(el);
  ce = cos(el);
  sp = sin(phi);
  cp = cos(phi);

  /*  HA,Dec as x,y,z */
  x = -ca * ce * sp + se * cp;
  y = -sa * ce;
  z = ca * ce * cp + se * sp;

  /*  To HA,Dec */
  r = sqrt(x * x + y * y);
  if (r == 0.) {
    *ha = 0.;
  } else {
    *ha = atan2(y, x);
  }
  *dec = atan2(z, r);

  return;
}

// phi = 22.2783203
void DecHaToAltAz ( double ha, double dec, double phi,
                    double *az, double *azd, double *azdd,
                    double *el, double *eld, double *eldd,
                    double *pa, double *pad, double *padd ) {

  /*
                 Arguments:
        ha = double (Given)
           Hour angle (radians)
        dec = double (Given)
           Declination (radians)
        phi = double (Given)
           Observatory latitude (radians)
        az = double * (Returned)
           Azimuth (radians)
        azd = double * (Returned)
           Azimuth velocity (radians per radian of HA)
        azdd = double * (Returned)
           Azimuth acceleration (radians per radian of HA squared)
        el = double * (Returned)
           Elevation (radians)
        eld = double * (Returned)
           Elevation velocity (radians per radian of HA)
        eldd = double * (Returned)
           Elevation acceleration (radians per radian of HA squared)
        pa = double * (Returned)
           Parallactic angle (radians)
        pad = double * (Returned)
           Parallactic angle velocity (radians per radian of HA)
        padd = double * (Returned)
           Parallactic angle acceleration (radians per radian of HA squared)
  */

  const double TINY = 1E-30;

  double sh, ch, sd, cd, sp, cp, chcd, sdcp, x, y, z, rsq, r, a, e, c, s,
         q, qd, ad, ed, edr, add, edd, qdd;


  /*  Useful functions */
  sh = sin(ha);
  ch = cos(ha);
  sd = sin(dec);
  cd = cos(dec);
  sp = sin(phi);
  cp = cos(phi);
  chcd = ch * cd;
  sdcp = sd * cp;
  x = -chcd * sp + sdcp;
  y = -sh * cd;
  z = chcd * cp + sd * sp;
  rsq = x * x + y * y;
  r = sqrt(rsq);

  /*  Azimuth and elevation */
  if (rsq == 0.0) {
    a = 0.0;
  } else {
    a = atan2(y, x);
  }
  if (a < 0.0) a += TWO_PI ;
  e = atan2(z, r);

  /*  Parallactic angle */
  c = cd * sp - ch * sdcp;
  s = sh * cp;
  if (c * c + s * s > 0) {
    q = atan2(s, c);
  } else {
    q = PI  - ha;
  }

  /*  Velocities and accelerations (clamped at zenith/nadir) */
  if (rsq < TINY) {
    rsq = TINY;
    r = sqrt(rsq);
  }
  qd = -x * cp / rsq;
  ad = sp + z * qd;
  ed = cp * y / r;
  edr = ed / r;
  add = edr * (z * sp + (2.0 - rsq) * qd);
  edd = -r * qd * ad;
  qdd = edr * (sp + 2.0 * z * qd);

  /*  Results */
  *az = a;
  *azd = ad;
  *azdd = add;
  *el = e;
  *eld = ed;
  *eldd = edd;
  *pa = q;
  *pad = qd;
  *padd = qdd;

}
