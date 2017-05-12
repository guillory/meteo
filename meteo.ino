// **********************************************************************************
// 
// **********************************************************************************
#include <ESP8266WiFi.h>
WiFiClient client;
#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

const byte DATAPINRAIN = D1;
const byte DATAPIN=D2;
const byte SDAPIN=D3;
const byte SLCPIN=D4;
const byte LEDPIN=D5;
const byte BMP180PIN=D7;
volatile boolean TX20IncomingData = false;
double Temperature;
double Pression;
double Humidite;

// domoticz
const char * domoticz_server = ""; //Domoticz port
int port = ;                          //Domoticz port
const char* ssid     = "";    
const char* password = "";
int debug=1;
double rssi ;
int  rssi_dec;
unsigned int    POST_INTERVAL =30 * 60 * 1000;   //30 minutes
unsigned long   previousMillisPOST =0;
unsigned long   previousHour =0;
double moyennewind_speed;
const int NB_SAMPLE = 900 ; // = POST_INTERVAL / (2* 1000) ; //  TX20 transmit data every two seconds 
int  wind_speed_array[NB_SAMPLE];
int  wind_direction_count_array[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // pour le cumul des directions
char wind_direction_libel[16][4] = {"N","NNE","NE", "ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"};
int  wind_direction_code;
int nbwifitry = 0;
int mesure = 0;
int mesure_positif=0;

int nbrainbucket = 1; // on compte que les pairs
int nbrainbuckethour = 1;
ADC_MODE(ADC_VCC);
double vdd;
/* ======================================================================
Function: setup
====================================================================== */
void setup(){  
  Serial.print("Start"); 
  pinMode(LEDPIN, OUTPUT);  digitalWrite(LEDPIN, LOW);    
  
  pinMode(DATAPIN, INPUT); 
  attachInterrupt(digitalPinToInterrupt(DATAPIN), readTX20, RISING);
  pinMode(DATAPINRAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor
  attachInterrupt(digitalPinToInterrupt(DATAPINRAIN), rainInterruptFunc, FALLING);
  interrupts();
  Serial.begin(115200);
  
  pinMode(BMP180PIN, OUTPUT);  digitalWrite(BMP180PIN, LOW);    

  inid_data();
  blink(); blink();  blink(); blink(); delay(1000);
 
 
}

/* ======================================================================
Function: loop
====================================================================== */
void loop(){

  if (TX20IncomingData) {
          char a[90];
          unsigned char chk;
          int bitcount=0;
          unsigned char sa,sb,sd,se;
          unsigned int sc,sf, pin;
      
          String message = "";
      
          sa=sb=sd=se=0;
          sc=0;sf=0;
      
          for (bitcount=41; bitcount>0; bitcount--) {
            pin = (digitalRead(DATAPIN));
            if (!pin) {  message += "1";} else {  message += "0";  }
            if ((bitcount==41-4) || (bitcount==41-8) || (bitcount==41-20)  || (bitcount==41-24)  || (bitcount==41-28)) {
              message += " ";
            }      
            if (bitcount > 41-5)              {sa = (sa<<1)|(pin^1);        } // start, inverted
            else if (bitcount > 41-5-4)       {sb = sb>>1 | ((pin^1)<<3);   } // wind dir, inverted
            else if (bitcount > 41-5-4-12)    {sc = sc>>1 | ((pin^1)<<11);  } // windspeed, inverted
            else if (bitcount > 41-5-4-12-4)  {sd = sd>>1 | ((pin^1)<<3);   } // checksum, inverted
            else if (bitcount > 41-5-4-12-4-4){se = se>>1 | (pin<<3);       } // wind dir
            else                              {sf = sf>>1 | (pin<<11);      } // windspeed
            delayMicroseconds(1220);    
          }
          if (debug){  Serial.println(message);}
          chk= ( sb + (sc&0xf) + ((sc>>4)&0xf) + ((sc>>8)&0xf) );chk&=0xf;
          if (debug){  
            sprintf(a, "ID: %d\t%d\n", sa, B00100);         Serial.write (a);
            sprintf(a, "Wind direction: %d\t%d\n", sb, se); Serial.write (a);
            sprintf(a, "Wind speed: %d\t%d\n", sc, sf);     Serial.write (a);
            sprintf(a, "Checksum: %d\t%d\n", sd, chk);      Serial.write (a);
          }
          if (sa==4 && sb==se && sc==sf && sd==chk){      
           if (debug){   Serial.println(" :) OK :) OK :) OK :) OK");}
           blink();
             mesure++;
             wind_speed_array[mesure % NB_SAMPLE]=sc;
             wind_direction_count_array[sb]++;
           } else {
           if (debug){   Serial.println(" !!! ERROR !!! ERROR !!!");}
          }
          
          delayMicroseconds(2000);  // just in case
          TX20IncomingData = false;  
  }
  
  unsigned long currentMillisPOST = millis();
  unsigned long currentHour = millis();
 
  // Timer pour lancer la publication à DOMOTICZ
  if((currentMillisPOST - previousMillisPOST) >= (POST_INTERVAL) ) {       // Garde en mémoire la valeur actuelle de millis()
      previousMillisPOST = currentMillisPOST;
      if (debug){  Serial.print("Connecting to ");  Serial.println(ssid); }
      nbwifitry=0;
      WiFi.mode(WIFI_STA);  
  	  WiFi.begin(ssid, password);
  	  while (WiFi.status() != WL_CONNECTED  and nbwifitry<=30) {
    	    delay(1000);
    	    Serial.print("."); blink();
          nbwifitry++;
  	  }
  	 if (WiFi.status() != WL_CONNECTED){// au bout de 60 seconde pas de connection.....
          if (debug){ Serial.println("");  Serial.println("PAS DE WIFI");   }
          delay(1000);
     }else{
        if (debug){ Serial.println("");  Serial.println("WiFi connected");    Serial.println("IP address: ");  Serial.println(WiFi.localIP());}

         // ---------------- BMP 180  // TODO GERER l'alimentation de la sonde. 
          digitalWrite(BMP180PIN, HIGH);  delay(2000);  
          Wire.begin (SDAPIN, SLCPIN); delay(2000);
          if (!bmp.begin())   {    Serial.println("Could not find BMP180 or BMP085 sensor at 0x77");  }
         
          Humidite=50;
          Temperature= bmp.readTemperature();
          Pression= bmp.readPressure();
          digitalWrite(BMP180PIN, LOW);
          if (debug){ Serial.print("Temperature = ");    Serial.print(Temperature);    Serial.println(" Celsius");   }
          if (debug){ Serial.print("Pressure = ");    Serial.print(Pression);    Serial.println(" Pascal");             }
          
          // ----------------
          // calcul VDD 
           vdd=get_vbat();
           if (debug){   Serial.print("vdd :");  Serial.println( vdd); }
           if (vdd>100 or vdd<0) {vdd= 255;}else{  vdd=int(vdd /0.05);}
           if (debug){   Serial.print("vdd level:");  Serial.println( vdd); }
          // calcul RSSI WIFI
           rssi =WiFi.RSSI();
           rssi_dec =  int ( (- (rssi*rssi)  +  (15000 *rssi) + 1700000 )/100000  ) ; // entre  1 et 12
           Serial.print("RSSI:");  Serial.print(rssi_dec);  Serial.print("/10  : RSSI: db ");  Serial.println(rssi);
          // ----------------
          // moyenne de la vitesse du vent
           int wind_rafale=0;
      	    moyennewind_speed=0;

      	   for ( mesure = 0 ; mesure <= NB_SAMPLE ; mesure++)    {           
      	      if ( wind_speed_array[mesure] >= 0) {                 // seules les valeur positives sont comptées
      	         mesure_positif++;
      	         moyennewind_speed += wind_speed_array[mesure];        // calcul de la moyenne
      	         if (wind_speed_array[mesure]>wind_rafale){wind_rafale=wind_speed_array[mesure];} // calcul du max
      	      }
      	  	}
          moyennewind_speed = int(moyennewind_speed / mesure_positif) ;
          if (debug){   Serial.print("moyennewind_speed :");  Serial.println( moyennewind_speed); }
          if (debug){   Serial.print("wind_rafale :");  Serial.println( wind_rafale); }
          if (debug){   Serial.print("Windchill :");  Serial.println( Windchill(Temperature, wind_rafale/10)); } // wind_rafale en décimetre/ seconde

           // ----------------
           // Direction la plus fréquente
           int wind_direction_code=-1;
           int direction;
           for ( direction = 0 ; direction <16 ; direction++)    { 
               if (wind_direction_count_array[direction] > wind_direction_code){ wind_direction_code=direction;}
            }
           if (debug){  Serial.print("wind_direction_code & wind_direction_libel : ");  Serial.print(wind_direction_code); Serial.print("  "); Serial.println( wind_direction_libel[wind_direction_code]); }
          
           
           if (debug){  Serial.print("nbrainbucket mm :");  Serial.println(nbrainbucket/2*0.518); }
           if (debug){  Serial.print("nbrainbuckethour mm :");  Serial.println(nbrainbuckethour/2*0.518); }
         
           Serial.println(F("POST DOMOTICZ "));

           // TODO pas de gestion des retour HTTP. 
      //------------------- ANEMOMETRE
                   if (client.connect(domoticz_server,port)  ) {
                      client.print("GET /json.htm?type=command&param=udevice&idx=62&nvalue=0&svalue=");
                      client.print(wind_direction_code*22.5);                 //WB = Wind bearing (0-359)
                      client.print(";");
                      client.print(wind_direction_libel[wind_direction_code]);//WD = Wind direction (S, SW, NNW, etc.)
                      client.print(";");
                      client.print(moyennewind_speed);                    //WS = 10 * Wind speed [m/s]
                      client.print(";");
                      client.print(wind_rafale);                           //WG = 10 * Gust [m/s]
                      client.print(";");
                      client.print(Temperature);                           //WTemperature
                      client.print(";");
                      client.print(Windchill(Temperature, wind_rafale/10));                                       //Temperature Windchill
                      client.print("&battery=");
                      client.print(vdd);   
                      client.print("&rssi=");
                      client.print(rssi_dec);   
                      client.println(" HTTP/1.1");
                      client.print("Host: ");
                      client.print(domoticz_server);
                      client.print(":");
                      client.println(port);
                      client.println("User-Agent: Arduino-ethernet");
                      client.println("Connection: close");
                      client.println();
                      client.stop();
                      Serial.println("posted ANEMO 62");
                   }
      //------------------- PLUVIOMETRE
                   if (client.connect(domoticz_server,port)  ) {
                      client.print("GET /json.htm?type=command&param=udevice&idx=63&nvalue=0&svalue=");
                      client.print(nbrainbucket/2*0.518);                                         //RAINRATE
                      client.print(";");
                      client.print(nbrainbuckethour/2*0.518 );                                    //RAINCOUN
                      client.print("&battery=");
                      client.print(vdd);   
                      client.print("&rssi=");
                      client.print(rssi_dec);   
                      client.println(" HTTP/1.1");
                      client.print("Host: ");
                      client.print(domoticz_server);
                      client.print(":");
                      client.println(port);
                      client.println("User-Agent: Arduino-ethernet");
                      client.println("Connection: close");
                      client.println();
                      client.stop();
                      Serial.println("posted PLUVIO  63 ");
                      // si ca fait plus d'une heure on vide le compteur horaire
                        if((currentHour - previousHour) >= (3600000) ) {
                           previousHour = currentHour;
                           nbrainbuckethour;
                        }
                   }
      //------------------- TEMP PRESSION HUMID
                    if (client.connect(domoticz_server,port)  ) {
                      client.print("GET /json.htm?type=command&param=udevice&idx=64&nvalue=0&svalue=");
                      client.print(Temperature);                      //TEMP
                      client.print(";");
                      client.print(Humidite);                         //HUM
                      client.print(";");
                      client.print(HUM_STAT(Humidite));                                          //HUM_STAT 0=Normal -> 3=Wet
                      client.print(";");
                      client.print(Pression/100);                                         //BAR
                      client.print(";");
                      client.print(0);                                     //BAR_FOR 0 = No info
                      client.print(";");
                      client.print("&battery=");
                      client.print(vdd);   
                      client.print("&rssi=");
                      client.print(rssi_dec);       
                      client.println(" HTTP/1.1");
                      client.print("Host: ");
                      client.print(domoticz_server);
                      client.print(":");
                      client.println(port);
                      client.println("User-Agent: Arduino-ethernet");
                      client.println("Connection: close");
                      client.println();
                      client.stop();
                      Serial.println("posted TEMP BARO 64 ");
                   }
        
      //------------------- VDD
                    if (client.connect(domoticz_server,port)  ) {
                      client.print("GET /json.htm?type=command&param=udevice&idx=25&nvalue=0&svalue=");
                      client.print(get_vbat());                      //TEMP
                      client.print("&battery=");
                      client.print(vdd);   
                      client.print("&rssi=");
                      client.print(rssi_dec);       
                      client.println(" HTTP/1.1");
                      client.print("Host: ");
                      client.print(domoticz_server);
                      client.print(":");
                      client.println(port);
                      client.println("User-Agent: Arduino-ethernet");
                      client.println("Connection: close");
                      client.println();
                      client.stop();
                      Serial.println("posted Elec 25 ");
                   }
                    inid_data();
                   WiFi.disconnect();
       			 WiFi.mode(WIFI_OFF);
      }
    }
}

// ======================================================================
float Windchill(float T,float V){
if (T<10 and V>4.8){
	   return 13.12 + 0.6215*T - (11.37*pow(V,0.16)) + (0.3965*T*pow(V,0.16));
	   //=13,12+(0,6215*B5)-(11,37*PUISSANCE(B4;0,16))+(0,3965*B5*PUISSANCE(B4;0,16))
	   //is calculation is meant for air temperatures lower than 10 Degrees Celsius and wind speeds greater than 4.8 km/h.
   }
  return T;
}
// ======================================================================
int HUM_STAT(float H){
  if (H<=25){return 0;}
  if (H<=50){return 1;}
  if (H<=75){return 2;}
  return 3;
}
double get_vbat() {
  double vcc = ESP.getVcc();
  vcc = vcc / 1000.0;
  return vcc;
}
// -----------------------------------------------------------------------------
// On initialise à -1 le tableau de chaque capteur de tension
bool inid_data() {
    for ( mesure = 0 ; mesure < NB_SAMPLE ; mesure++)    {
      wind_speed_array[mesure] = -1;
      wind_direction_count_array[mesure]=0;
    }
  mesure = 0;
  nbrainbucket=1;
  mesure_positif=1;
  
}
// ======================================================================
void readTX20() {
  if (!TX20IncomingData) {    TX20IncomingData = true;  }  
}
// ======================================================================
void rainInterruptFunc(){
  nbrainbucket++;
  nbrainbuckethour++;
  
  if (debug){ Serial.print("nbrainbucket: "); Serial.println(nbrainbucket/2);}
  if (debug){ Serial.print("nbrainbuckethour: "); Serial.println(nbrainbuckethour/2);}
}
// ======================================================================
void blink(){
if (mesure<=10){ // on blink qu'au début
  digitalWrite(LEDPIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a 1/2 second
  digitalWrite(LEDPIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200); 
 }
}
