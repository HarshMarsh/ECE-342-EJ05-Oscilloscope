/* 
ECE 341 EJ05 O-scope project:
This program takes measurements from two ADC channels and 
stores them in buffers using DMA. This data is then printed
to the serial monitor when the buffer is full.

This sketch also utilizes an encoder that allows the user to manually 
adjust the sampling frequency of the ADC's. This is helpful for scaling the 
buffer to the input signal frequency


Example for triggering the ADC with Timer using DMA instead of interrupts
    Valid for the current Teensy 3.x and 4.0.


  Timers:
    On Teensy 3.x this uses the PDB timer.

    On Teensy 4, this uses one or two of the unused QTimers.

    Setting it up:  The variables readPin must be defined for a pin that is
  valid for the first (or only) ADC.  If the processor has a second ADC and is
  enabled, than readPin2 must be configured to be a pin that is valid on the
  second ADC.

  DMA: using AnalogBufferDMA with two buffers, this runs in continuous mode and
  when one buffer fills an interrupt is signaled, which sets flag saying it has
  data, which this test application scans the data, and computes things like a
  minimum, maximum, average values and an RMS value. For the RMS it keeps the
  average from the previous set of data.
*/

#include <ADC.h>
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached


long oldPosition  = 1000;

//Variables for low-pass filtering
const float alpha = 0.8; //<- Making this valu lower intensifies filtering effect and vice-versa
double data_filtered[] = { 0, 0 }; // data_filtered[n] is where the most recent filtered value can be accessed
const int n = 1;


#if defined(ADC_USE_DMA) && defined(ADC_USE_TIMER)

#include <AnalogBufferDMA.h>
#include <DMAChannel.h>
/*
// This version uses both ADC1 and ADC2
const int readPin_adc_0 = A0;
const int readPin_adc_1 = 26;
*/
// This version uses both ADC1 and ADC2
#if defined(KINETISL)
const int readPin_adc_0 = A0;
#elif defined(KINETISK)
const int readPin_adc_0 = A0;
const int readPin_adc_1 = A2;
#else
const int readPin_adc_0 = A0;
const int readPin_adc_1 = 26;
#endif

ADC *adc = new ADC(); // adc object
const uint32_t initial_average_value = 0;

extern void dumpDMA_structures(DMABaseClass *dmabc);
elapsedMillis elapsed_sinc_last_display;

// Going to try two buffers here  using 2 dmaSettings and a DMAChannel

const uint32_t buffer_size = 1200;
DMAMEM static volatile uint16_t __attribute__((aligned(32)))
dma_adc_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32)))
dma_adc_buff2[buffer_size];
AnalogBufferDMA abdma1(dma_adc_buff1, buffer_size, dma_adc_buff2, buffer_size);

//#ifdef ADC_DUAL_ADCS
//Serial.print("\nSetting up second buffer!");
DMAMEM static volatile uint16_t __attribute__((aligned(32)))
dma_adc_buff2_1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32)))
dma_adc_buff2_2[buffer_size];
AnalogBufferDMA abdma2(dma_adc_buff2_1, buffer_size, dma_adc_buff2_2,
                       buffer_size);
//#endif

void setup() {
  Serial.begin(1000000);
  while (!Serial && millis() < 5000)
    ;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin_adc_0, INPUT_DISABLE); // Not sure this does anything for us
//#ifdef ADC_DUAL_ADCS
  pinMode(readPin_adc_1, INPUT_DISABLE);
//#endif

  Serial.println("Setup both ADCs");
  // Setup both ADCs
  adc->adc0->setAveraging(1);   // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
//#ifdef ADC_DUAL_ADCS
  adc->adc1->setAveraging(1);   // set number of averages
  adc->adc1->setResolution(10); // set bits of resolution
//#endif

  // enable DMA and interrupts
  // Serial.println("before enableDMA"); Serial.flush();

  // setup a DMA Channel.
  // Now lets see the different things that RingbufferDMA setup for us before
  abdma1.init(adc, ADC_0 /*, DMAMUX_SOURCE_ADC_ETC*/);
  abdma1.userData(initial_average_value); // save away initial starting average
//#ifdef ADC_DUAL_ADCS
  abdma2.init(adc, ADC_1 /*, DMAMUX_SOURCE_ADC_ETC*/);
  abdma2.userData(initial_average_value); // save away initial starting average
//#endif
  // Serial.println("After enableDMA"); Serial.flush();

  // Start the dma operation..
  adc->adc0->startSingleRead(
      readPin_adc_0); // call this to setup everything before the Timer starts,
                      // differential is also possible
 // adc->adc0->startTimer(10000); // frequency in Hz
  adc->adc0->startTimer(100000); // frequency in Hz

  // Start the dma operation..
//#ifdef ADC_DUAL_ADCS
  adc->adc1->startSingleRead(
      readPin_adc_1); // call this to setup everything before the Timer starts,
                      // differential is also possible
 // adc->adc1->startTimer(10000); // frequency in Hz
 adc->adc1->startTimer(100000); // frequency in Hz
//#endif

  print_debug_information();

  Serial.println("End Setup");
  elapsed_sinc_last_display = 0;
}

void loop() {

    int max = 5000;
  int min = 1;
  long newPosition = myEnc.read();
  if(newPosition > max){
    newPosition = max;
  }else if(newPosition < min){
    newPosition = min;
  }
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition*100);
    reconfigure(newPosition*100);
    Serial.print("\nChange happened!");
  }

  // Maybe only when both have triggered?
//#ifdef ADC_DUAL_ADCS
  if (abdma1.interrupted() && (abdma2.interrupted())) {
    if (abdma1.interrupted()) {
      ProcessAnalogData(&abdma1, 0);
    }
    if (abdma2.interrupted()) {
      ProcessAnalogData(&abdma2, 1);
    }
    Serial.println();
    elapsed_sinc_last_display = 0;
  }
// #else
//   if (abdma1.interrupted()) {
//     ProcessAnalogData(&abdma1, 0);
//     Serial.println();
//     elapsed_sinc_last_display = 0;
//   }
// #endif
  if (elapsed_sinc_last_display > 5000) {
    // Nothing in 5 seconds, show a heart beat.
    digitalWriteFast(13, HIGH);
    delay(250);
    digitalWriteFast(13, LOW);
    delay(250);
    digitalWriteFast(13, HIGH);
    delay(250);
    digitalWriteFast(13, LOW);
    elapsed_sinc_last_display = 0;
  }
}

void ProcessAnalogData(AnalogBufferDMA *pabdma, int8_t adc_num) {
   uint16_t min_val = 0xffff;
  uint16_t max_val = 0;
  volatile uint16_t adc_1_var;
  // uint32_t average_value = pabdma->userData();

  volatile uint16_t *pbuffer = pabdma->bufferLastISRFilled();
  volatile uint16_t *end_pbuffer = pbuffer + pabdma->bufferCountLastISRFilled();
  //Serial.print("\nBuffer Contents:");
  //float sum_delta_sq = 0.0;
  if ((uint32_t)pbuffer >= 0x20200000u)
    arm_dcache_delete((void *)pbuffer, sizeof(dma_adc_buff1));
  while (pbuffer < end_pbuffer) {
    if (*pbuffer < min_val)
      min_val = *pbuffer;
    if (*pbuffer > max_val)
      max_val = *pbuffer;
    //read_sensor(*pbuffer);
    //Serial.println(data_filtered[n]);
    Serial.println(((*pbuffer* -1)*3.3)/4096);
    pbuffer++;
  }

  pabdma->clearInterrupt();

  //pabdma->userData(average_value);
}

void print_debug_information() {
#ifdef PRINT_DEBUG_INFO
  // Lets again try dumping lots of data.
  Serial.println("\n*** DMA structures for ADC_0 ***");
  dumpDMA_structures(&(abdma1._dmachannel_adc));
  dumpDMA_structures(&(abdma1._dmasettings_adc[0]));
  dumpDMA_structures(&(abdma1._dmasettings_adc[1]));
  Serial.println("\n*** DMA structures for ADC_1 ***");
  dumpDMA_structures(&(abdma2._dmachannel_adc));
  dumpDMA_structures(&(abdma2._dmasettings_adc[0]));
  dumpDMA_structures(&(abdma2._dmasettings_adc[1]));

#if defined(__IMXRT1062__)

  Serial.println("\n*** ADC and ADC_ETC ***");
  Serial.printf("ADC1: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", IMXRT_ADC1.HC0,
                IMXRT_ADC1.HS, IMXRT_ADC1.CFG, IMXRT_ADC1.GC, IMXRT_ADC1.GS);
  Serial.printf("ADC2: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", IMXRT_ADC2.HC0,
                IMXRT_ADC2.HS, IMXRT_ADC2.CFG, IMXRT_ADC2.GC, IMXRT_ADC2.GS);
  Serial.printf("ADC_ETC: CTRL:%x DONE0_1:%x DONE2_ERR:%x DMA: %x\n",
                IMXRT_ADC_ETC.CTRL, IMXRT_ADC_ETC.DONE0_1_IRQ,
                IMXRT_ADC_ETC.DONE2_ERR_IRQ, IMXRT_ADC_ETC.DMA_CTRL);
  for (uint8_t trig = 0; trig < 8; trig++) {
    Serial.printf("    TRIG[%d] CTRL: %x CHAIN_1_0:%x\n", trig,
                  IMXRT_ADC_ETC.TRIG[trig].CTRL,
                  IMXRT_ADC_ETC.TRIG[trig].CHAIN_1_0);
  }
#endif
#endif
}

#ifdef PRINT_DEBUG_INFO
void dumpDMA_structures(DMABaseClass *dmabc) {
  Serial.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf(
      "SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n",
      (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
      dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
      dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
      dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

#else  // make sure the example can run for any boards (automated testing)
void setup() {}
void loop() {}
#endif // ADC_USE_TIMER and DMA


void reconfigure(int speed)
{
   adc->adc0->setAveraging(1);   // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
//#ifdef ADC_DUAL_ADCS
  adc->adc1->setAveraging(1);   // set number of averages
  adc->adc1->setResolution(10); // set bits of resolution
//#endif

  // enable DMA and interrupts
  // Serial.println("before enableDMA"); Serial.flush();

  // setup a DMA Channel.
  // Now lets see the different things that RingbufferDMA setup for us before
  abdma1.init(adc, ADC_0 /*, DMAMUX_SOURCE_ADC_ETC*/);
  abdma1.userData(initial_average_value); // save away initial starting average
//#ifdef ADC_DUAL_ADCS
  abdma2.init(adc, ADC_1 /*, DMAMUX_SOURCE_ADC_ETC*/);
  abdma2.userData(initial_average_value); // save away initial starting average
//#endif
  // Serial.println("After enableDMA"); Serial.flush();

  // Start the dma operation..
  adc->adc0->startSingleRead(
      readPin_adc_0); // call this to setup everything before the Timer starts,
                      // differential is also possible
 // adc->adc0->startTimer(10000); // frequency in Hz
  adc->adc0->startTimer(speed); // frequency in Hz

  // Start the dma operation..
//#ifdef ADC_DUAL_ADCS
  adc->adc1->startSingleRead(
      readPin_adc_1); // call this to setup everything before the Timer starts,
                      // differential is also possible
 // adc->adc1->startTimer(10000); // frequency in Hz
 adc->adc1->startTimer(speed); // frequency in Hz
//#endif

  print_debug_information();

  Serial.println("End Setup");
  elapsed_sinc_last_display = 0;
}
void read_sensor(int data_in) {
  // if the received serial command tells us to send sensor data, take a measurement from the photodiode,
  // pass it through the lowpass filter, and send it back to the PC/
  //  if (c == send_data) {
  // Retrieve Data
  

  // Low Pass Filter
  data_filtered[n] = alpha *  data_in + (1 - alpha) * data_filtered[n - 1];

  // Store the last filtered data in data_filtered[n-1]
  data_filtered[n - 1] = data_filtered[n];

}