#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <ADC.h>
#include <AnalogBufferDMA.h>
#include <DMAChannel.h>
#include <Encoder.h>

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

#define COLUMNS 15
#define ROWS 11

long vScale = 250;
long hScale = 40;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, 11, 13, 6, 12);

#if defined(ADC_USE_DMA) && defined(ADC_USE_TIMER)

const int readPin_adc_0 = 14;
const int readPin_adc_1 = 15;

ADC *adc = new ADC();

const uint32_t buffer_size = 500;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[buffer_size];
AnalogBufferDMA abdma1(dma_adc_buff1, buffer_size, dma_adc_buff2, buffer_size);

DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2_2[buffer_size];
AnalogBufferDMA abdma2(dma_adc_buff2_1, buffer_size, dma_adc_buff2_2, buffer_size);

elapsedMillis elapsed_sinc_last_display;


Encoder trigger_Enc(21, 22);
Encoder scale_Enc(19, 20);
#define scale_select 18

void setup() {
  pinMode(scale_select, INPUT);
  Serial.begin(1000000);
  while (!Serial && millis() < 5000)
    ;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin_adc_0, INPUT_DISABLE);
  pinMode(readPin_adc_1, INPUT_DISABLE);

  adc->adc0->setAveraging(1);
  adc->adc0->setResolution(10);
  adc->adc1->setAveraging(1);
  adc->adc1->setResolution(10);

  abdma1.init(adc, ADC_0);
  abdma1.userData(0);
  abdma2.init(adc, ADC_1);
  abdma2.userData(0);

  adc->adc0->startSingleRead(readPin_adc_0);
  adc->adc0->startTimer(400000);
  adc->adc1->startSingleRead(readPin_adc_1);
  adc->adc1->startTimer(400000);

  tft.begin();
  tft.setRotation(1);

  drawFrame();
}

void loop() {
  if (abdma1.interrupted() && abdma2.interrupted()) {
    ProcessAnalogData(&abdma1, 0,&abdma2, 1);
 //   ProcessAnalogData(&abdma2, 1);
    elapsed_sinc_last_display = 0;
  }

  if (elapsed_sinc_last_display > 5000) {
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

void ProcessAnalogData(AnalogBufferDMA *pabdma, int8_t adc_num1,AnalogBufferDMA *pabdma2, int8_t adc_num2) {
    //long hscale;
    //long vscale
    long trigger_level = trigger_Enc.read();
    if(digitalRead(scale_select)){
     vScale = scale_Enc.read();
    }else{
      hScale = scale_Enc.read();
    }

    Serial.print("\nvscale: ");
    Serial.print(vScale);
    Serial.print(", hscale: ");
    Serial.print(hScale);

      volatile uint16_t *pbuffer2 = pabdma2->bufferLastISRFilled();
  volatile uint16_t *end_pbuffer2 = pbuffer2 + pabdma->bufferCountLastISRFilled();
        
  volatile uint16_t *pbuffer = pabdma->bufferLastISRFilled();
  volatile uint16_t *end_pbuffer = pbuffer + pabdma->bufferCountLastISRFilled();


    if ((uint32_t)pbuffer2 >= 0x20200000u) {
    arm_dcache_delete((void *)pbuffer, sizeof(dma_adc_buff1));
  }


  if ((uint32_t)pbuffer >= 0x20200000u) {
    arm_dcache_delete((void *)pbuffer, sizeof(dma_adc_buff1));
  }



  tft.fillRect(0, 0, tft.width(), tft.height(), ILI9341_BLACK);
  drawFrame();

  int x = 0;
  while (pbuffer < end_pbuffer && x < tft.width()) {
    //Serial.println(*pbuffer);
    int y1 = tft.height() / 2 - (*pbuffer * 10 / vScale);
    int y2 = tft.height() / 2 - (*pbuffer2 * 10 / vScale);
   // if(adc_num == 0){
    tft.drawPixel(x, y1, ILI9341_GREEN);
    tft.drawPixel(x, y2, ILI9341_BLUE);
    //}else{
    //   tft.drawPixel(x, y, ILI9341_BLUE);
    //}
    delayMicroseconds(1000);
    //delay(1);
    //Serial.print("\nx:");
    //Serial.print(x);
    //Serial.print(" , y: ");
    //Serial.print(y);
    pbuffer++;
    pbuffer2++;
    x++;
  }

  pabdma->clearInterrupt();
  pabdma2->clearInterrupt();
}

void drawFrame() {
  tft.fillRect(0, 0, tft.width(), tft.height(), ILI9341_BLACK);
  for (int16_t i = 0; i < COLUMNS; i++) {
    for (int16_t j = 0; j < ROWS; j++) {
      tft.drawFastVLine(20 * i, j * 20 - 2, 5, ILI9341_WHITE);
      tft.drawFastHLine(20 * i - 2, j * 20, 5, ILI9341_WHITE);
    }
  }

  tft.setCursor(0, ROWS * 20);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.print("V Scale: ");
  tft.println(vScale);
  tft.setCursor(100, ROWS * 20);
  tft.print("H Scale: ");
  tft.setCursor(100, ROWS * 20 + 8);
  tft.println(hScale);
}

#else
void setup() {}
void loop() {}
#endif
