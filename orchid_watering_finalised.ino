#define BLYNK_TEMPLATE_ID           "TMPL6jVh9oc9x"
#define BLYNK_TEMPLATE_NAME         "Watering"
#define BLYNK_AUTH_TOKEN            "9ylm9dT5s7-kOGGXGJwnrdvKta7I4js1"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <Adafruit_Sensor.h>
#include "DHT.h"
//include "fis_header.h"
#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

char ssid[] = "hafiedz";
char pass[] = "123pass123";

typedef FIS_TYPE(_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(_FIS_ARR)(FIS_TYPE, int, _FIS_ARR_OP);

#define PUMP 26 
#define DHTPIN 4
#define DHTTYPE DHT22 
#define SOIL 34
#define TANK 35
#define TANKPIN 13
#define SOILPIN 17
BlynkTimer timer; 

DHT dht(DHTPIN, DHTTYPE);
int val = 0;

FIS_TYPE fis_trapmf(FIS_TYPE, FIS_TYPE*);
_FIS_MF* fis_gMF[] = {fis_trapmf};

// Number of inputs to the fuzzy inference system
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 27;
FIS_TYPE g_fisInput[3];
FIS_TYPE g_fisOutput[1];

void setup() {
  pinMode(PUMP, OUTPUT);
  pinMode(TANKPIN, OUTPUT);
  pinMode (SOILPIN,OUTPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  dht.begin();
  Serial.begin(115200);
  delay(5000);
  Serial.println(F("DHTxx test!"));
  waterPlants();
  delay(2000);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(5000L, updateSensorValues);
  
}

int readWaterLevel (){
  digitalWrite(TANKPIN, HIGH);
  delay(10);
  int tankval=analogRead(TANK);
  int mappedtankval = map(tankval, 340, 1557, 1, 100);
  digitalWrite(TANKPIN,LOW);
  return mappedtankval;
}

float readSoilMoisture(){
   int soilval=analogRead(SOIL);
   int mappedsoilval = map(soilval, 2300, 4095, 100, 1);
   return mappedsoilval;
}

void updateSensorValues() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int waterLevel = readWaterLevel();
  int soilMoisture = readSoilMoisture();
  
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, humidity);
  Blynk.virtualWrite(V3, waterLevel);
  Blynk.virtualWrite(V4, soilMoisture);
  Blynk.virtualWrite(V5, LOW); // Initialize the button state as OFF
  Blynk.syncVirtual(V5); 
}

BLYNK_WRITE(V5) {
  int buttonState = param.asInt();
  
  if (buttonState == 1) {
    // The button is pressed, run the pump for 2 seconds
    Serial.println("button is pressed");
    digitalWrite(PUMP, HIGH); // Turn on the pump
    delay(3000);                 // Run the pump for 2 seconds
    digitalWrite(PUMP, LOW);  // Turn off the pump
    Blynk.virtualWrite(V5, LOW);  // Set the button state back to OFF
  }
}

void waterPlants() {
  float temperatureSum = 0;
  float humiditySum = 0;
  int waterLevelSum = 0;
  int soilMoistureSum = 0;
  int readings = 3;  // Number of readings to average (6 * 10 seconds = 60 seconds)

  for (int i = 0; i < readings; i++) {
    // Read sensor values
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int waterLevel = readWaterLevel();
    int soilMoisture = readSoilMoisture();

    // Accumulate readings
    temperatureSum += temperature;
    humiditySum += humidity;
    waterLevelSum += waterLevel;
    soilMoistureSum += soilMoisture;

    // Delay for 5 seconds
    delay(5000);
  }

  // Calculate average values
  float averageTemperature = temperatureSum / readings;
  float averageHumidity = humiditySum / readings;
  int averageWaterLevel = waterLevelSum / readings;
  int averageSoilMoisture = soilMoistureSum / readings;

  // Print the average values
  Serial.println("Average Moisture Percentage: ");
  Serial.println(averageSoilMoisture);
  Serial.println("Average Temperature(°C): ");
  Serial.println(averageTemperature);
  Serial.println("Average Humidity(%): ");
  Serial.println(averageHumidity);
  Serial.println("Average Water Level: ");
  Serial.println(averageWaterLevel);

  // Read Input: Temperature
  g_fisInput[0] = averageTemperature;
  // Read Input: Humidity
  g_fisInput[1] = averageHumidity;
  // Read Input: SoilMoisture
  g_fisInput[2] = averageSoilMoisture;

  g_fisOutput[0] = 0;

  fis_evaluate();
 
  int outputDuration = int(g_fisOutput[0] * 1000); // Convert to milliseconds

  // Set output value: WaterFlow (assuming pin 3 is a pump)
  digitalWrite(PUMP, HIGH); // Turn on the pump
  delay(outputDuration); // Keep the pump on for the calculated duration
  digitalWrite(PUMP, LOW); // Turn off the pump

  // Print the fuzzy logic output and corresponding duration
  Serial.print("Fuzzy Logic Output: ");
  Serial.println(g_fisOutput[0]);
  Serial.print("Output Duration (ms): ");
  Serial.println(outputDuration);

}

void loop(){
// Variables to accumulate sensor readings and count iterations
  Blynk.run();
  timer.run(); 
    // Add your custom logic to trigger watering and control timing here
  static unsigned long previousWateringTime = 0;
  unsigned long currentTime = millis();
  unsigned long wateringInterval = 43200000L;  // 12 hours in milliseconds
  
  if (currentTime - previousWateringTime >= wateringInterval) {
    // It's time to water the plants, call the waterPlants function
    waterPlants();
    previousWateringTime = currentTime;  // Update the previous watering time
  }
}

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 0, 15, 20 };
FIS_TYPE fis_gMFI0Coeff2[] = { 15, 20, 25, 30 };
FIS_TYPE fis_gMFI0Coeff3[] = { 25, 30, 100, 100 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 50, 55 };
FIS_TYPE fis_gMFI1Coeff2[] = { 50, 55, 65, 70 };
FIS_TYPE fis_gMFI1Coeff3[] = { 65, 70, 100, 100 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 15, 20 };
FIS_TYPE fis_gMFI2Coeff2[] = { 15, 20, 35, 40 };
FIS_TYPE fis_gMFI2Coeff3[] = { 35, 40, 100, 100 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 3, 4 };
FIS_TYPE fis_gMFO0Coeff2[] = { 3, 4, 6, 7 };
FIS_TYPE fis_gMFO0Coeff3[] = { 6, 7, 10, 10 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1 };
int fis_gRI1[] = { 1, 1, 2 };
int fis_gRI2[] = { 1, 1, 3 };
int fis_gRI3[] = { 1, 2, 1 };
int fis_gRI4[] = { 1, 2, 2 };
int fis_gRI5[] = { 1, 2, 3 };
int fis_gRI6[] = { 1, 3, 1 };
int fis_gRI7[] = { 1, 3, 2 };
int fis_gRI8[] = { 1, 3, 3 };
int fis_gRI9[] = { 2, 1, 1 };
int fis_gRI10[] = { 2, 1, 2 };
int fis_gRI11[] = { 2, 1, 3 };
int fis_gRI12[] = { 2, 2, 1 };
int fis_gRI13[] = { 2, 2, 2 };
int fis_gRI14[] = { 2, 2, 3 };
int fis_gRI15[] = { 2, 3, 1 };
int fis_gRI16[] = { 2, 3, 2 };
int fis_gRI17[] = { 2, 3, 3 };
int fis_gRI18[] = { 3, 1, 1 };
int fis_gRI19[] = { 3, 1, 2 };
int fis_gRI20[] = { 3, 1, 3 };
int fis_gRI21[] = { 3, 2, 1 };
int fis_gRI22[] = { 3, 2, 2 };
int fis_gRI23[] = { 3, 2, 3 };
int fis_gRI24[] = { 3, 3, 2 };
int fis_gRI25[] = { 3, 3, 1 };
int fis_gRI26[] = { 3, 3, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19, fis_gRI20, fis_gRI21, fis_gRI22, fis_gRI23, fis_gRI24, fis_gRI25, fis_gRI26 };

// Rule Outputs
int fis_gRO0[] = { 3 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 1 };
int fis_gRO3[] = { 3 };
int fis_gRO4[] = { 2 };
int fis_gRO5[] = { 1 };
int fis_gRO6[] = { 2 };
int fis_gRO7[] = { 1 };
int fis_gRO8[] = { 1 };
int fis_gRO9[] = { 3 };
int fis_gRO10[] = { 2 };
int fis_gRO11[] = { 1 };
int fis_gRO12[] = { 3 };
int fis_gRO13[] = { 2 };
int fis_gRO14[] = { 1 };
int fis_gRO15[] = { 2 };
int fis_gRO16[] = { 1 };
int fis_gRO17[] = { 1 };
int fis_gRO18[] = { 3 };
int fis_gRO19[] = { 3 };
int fis_gRO20[] = { 2 };
int fis_gRO21[] = { 3 };
int fis_gRO22[] = { 2 };
int fis_gRO23[] = { 1 };
int fis_gRO24[] = { 1 };
int fis_gRO25[] = { 2 };
int fis_gRO26[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19, fis_gRO20, fis_gRO21, fis_gRO22, fis_gRO23, fis_gRO24, fis_gRO25, fis_gRO26 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 50, 100, 100 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 10 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}


//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }
    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
