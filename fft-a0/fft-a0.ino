#include <arduinoFFT.h>

#define SAMPLES 128            // Must be a power of 2
#define SAMPLING_FREQUENCY 1 // Hz, change this to match your sensor's sampling rate
#define A0_PIN A0

arduinoFFT FFT = arduinoFFT();

double vReal[SAMPLES];
double vImag[SAMPLES];
float sample_rate = 0;
int elapsedTime = 0;
void setup() {
  Serial.begin(115200);

  // Record the start time
  int startTime = micros();

  // Read the analog input
  int sensorValue = analogRead(A0_PIN);

  // Record the end time
  int endTime = micros();

  // Calculate the elapsed time
  elapsedTime = endTime - startTime;
  sample_rate = SAMPLING_FREQUENCY / (1.5 * elapsedTime*1e-6);
  Serial.println(sample_rate);
  delay(1000);
}
void loop() {
  // Read analog input and fill the vReal array
  float sample_sum = 0;
  float del = 1/sample_rate;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(A0_PIN);
    sample_sum = sample_sum + vReal[i];
    vImag[i] = 0;
    delayMicroseconds(elapsedTime/3);
  }
  float sample_mean = sample_sum / SAMPLES;
  for (int i = 0; i < SAMPLES; i++)
    vReal[i] = vReal[i] - sample_mean;
  // Perform FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // Find the frequency with the highest magnitude
  double maxMagnitude = 0;
  int maxIndex = 0;

  for (int i = 0; i < ((SAMPLES+1)/2); i++) {
    if (vReal[i] > maxMagnitude) {
      maxMagnitude = vReal[i];
      maxIndex = i;
    }
  }

  // Calculate the frequency
  float frequency = (maxIndex * sample_rate*1.5) / (SAMPLES);

  // Print the result
  // Serial.print("Frequency with highest magnitude: ");
  Serial.println(frequency);
  // Serial.println(" Hz");

  // delay(1000); // Adjust delay as needed
}
