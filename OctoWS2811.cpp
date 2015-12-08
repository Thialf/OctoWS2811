/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include <string.h>
#include "OctoWS2811.h"

uint16_t OctoWS2811::stripLen;
void * OctoWS2811::frameBuffer;
void * OctoWS2811::drawBuffer;
uint8_t OctoWS2811::params;
DMAChannel OctoWS2811::dma1;
DMAChannel OctoWS2811::dma2;
DMAChannel OctoWS2811::dma3;

static const uint8_t ones = 0xFF;
static volatile uint8_t update_in_progress = 0;
static uint32_t update_completed_at = 0;


OctoWS2811::OctoWS2811(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config)
{
	stripLen = numPerStrip;
	frameBuffer = frameBuf;
	drawBuffer = drawBuf;
	params = config;
}

// Waveform timing: these set the high time for a 0 and 1 bit, as a fraction of
// the total 800 kHz or 400 kHz clock cycle.  The scale is 0 to 255.  The Worldsemi
// datasheet seems T1H should be 600 ns of a 1250 ns cycle, or 48%.  That may
// erroneous information?  Other sources reason the chip actually samples the
// line close to the center of each bit time, so T1H should be 80% if TOH is 20%.
// The chips appear to work based on a simple one-shot delay triggered by the
// rising edge.  At least 1 chip tested retransmits 0 as a 330 ns pulse (26%) and
// a 1 as a 660 ns pulse (53%).  Perhaps it's actually sampling near 500 ns?
// There doesn't seem to be any advantage to making T1H less, as long as there
// is sufficient low time before the end of the cycle, so the next rising edge
// can be detected.  T0H has been lengthened slightly, because the pulse can
// narrow if the DMA controller has extra latency during bus arbitration.  If you
// have an insight about tuning these parameters AND you have actually tested on
// real LED strips, please contact paul@pjrc.com.  Please do not email based only
// on reading the datasheets and purely theoretical analysis.
// Fastled: T0H=280ns, T1H=580ns, all=1.33µs
#define WS2811_TIMING_T0H  60 //60
#define WS2811_TIMING_T1H  176 //176

// Discussion about timing and flicker & color shift issues:
// http://forum.pjrc.com/threads/23877-WS2812B-compatible-with-OctoWS2811-library?p=38190&viewfull=1#post38190

static uint8_t analog_write_res = 8;

// FTM is clocked by the bus clock, 48 MHz

#if F_BUS < 48000000
#error this OctoWS2811 is not working with F_BUS lower than 48MHz
#endif

void setFTM_Timer(uint8_t ch1, uint8_t ch2, float frequency)
{
	uint32_t prescale, mod, ftmClock, ftmClockSource;
	float minfreq;

  ftmClockSource = 1; 			//Use F_BUS clock source
  ftmClock = F_BUS;					//Set variable for the actual timer clock frequency

	for (prescale = 0; prescale < 7; prescale++) {
		minfreq = (float)(ftmClock >> prescale) / 65536.0f;
		if (frequency >= minfreq) break;
	}

	mod = (float)(ftmClock >> prescale) / frequency - 0.5f;
	if (mod > 65535) mod = 65535;

  FTM0_SC = 0; // stop FTM until setting of registers are ready
  FTM0_CNTIN = 0; // initial value for counter. CNT will be set to this value, if any value is written to FTMx_CNT 
  FTM0_CNT = 0;
  FTM0_MOD = mod;
  
  // I don't know why, but the following leads to a very short first pulse. Shifting the compare values to the end looks much better
  // uint32_t cval;
	// FTM0_C0V = 1;  // 0 is not working -> add 1 to every compare value.
  // cval = ((uint32_t)ch1 * (uint32_t)(mod + 1)) >> analog_write_res;
	// FTM0_C1V = cval +1;
  // cval = ((uint32_t)ch2 * (uint32_t)(mod + 1)) >> analog_write_res;
  // FTM0_C2V = cval +1;
  
  // Shifting the compare values to the end leads to a perfect first (and last) pulse:
  uint32_t cval1 = ((uint32_t)ch1 * (uint32_t)(mod + 1)) >> analog_write_res;
  uint32_t cval2 = ((uint32_t)ch2 * (uint32_t)(mod + 1)) >> analog_write_res;
  FTM0_C0V = mod - (cval2 - 0);
	FTM0_C1V = mod - (cval2 - cval1);
  FTM0_C2V = mod;

  FTM0_C0SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28;
  FTM0_C1SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28;
  FTM0_C2SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28;
    
  
  FTM0_SC = FTM_SC_CLKS(ftmClockSource) | FTM_SC_PS(prescale);	//Use ftmClockSource instead of 1. Start FTM-Timer.
  //with 96MHz Teensy: prescale 0, mod 59, ftmClockSource 1, cval1 14, cval2 41
}


void OctoWS2811::begin(char dataport, uint32_t datadirectionregister)
{
	uint32_t bufsize, frequency;

	bufsize = stripLen*24;

	// set up the buffers
	memset(frameBuffer, 0, bufsize);
	if (drawBuffer) {
		memset(drawBuffer, 0, bufsize);
	} else {
		drawBuffer = frameBuffer;
	}
	
	// configure the 8 output pins
  switch(dataport)
  {
    case 'A':
      GPIOA_PCOR |= datadirectionregister;
      if (datadirectionregister & (1<<4))
        pinMode(33, OUTPUT);      // strip #1
      if (datadirectionregister & (1<<5))
        pinMode(24, OUTPUT);      // strip #2
      break;
    case 'B':
      GPIOB_PCOR |= datadirectionregister;
      if (datadirectionregister & (1<<0))
        pinMode(16, OUTPUT);      // strip #1
      if (datadirectionregister & (1<<1))
        pinMode(17, OUTPUT);      // strip #2
      if (datadirectionregister & (1<<2))
        pinMode(19, OUTPUT);      // strip #3
      if (datadirectionregister & (1<<3))
        pinMode(18, OUTPUT);      // strip #4
      break;
    case 'C':
      GPIOC_PCOR |= datadirectionregister;
      if (datadirectionregister & (1<<0))
        pinMode(15, OUTPUT);      // strip #1
      if (datadirectionregister & (1<<1))
        pinMode(22, OUTPUT);      // strip #2
      if (datadirectionregister & (1<<2))
        pinMode(23, OUTPUT);      // strip #3
      if (datadirectionregister & (1<<3))
        pinMode( 9, OUTPUT);      // strip #4
      if (datadirectionregister & (1<<4))
        pinMode(10, OUTPUT);      // strip #5
      if (datadirectionregister & (1<<5))
        pinMode(13, OUTPUT);      // strip #6
      if (datadirectionregister & (1<<6))
        pinMode(11, OUTPUT);      // strip #7
      if (datadirectionregister & (1<<7))
        pinMode(12, OUTPUT);      // strip #8
      break;
    case 'E':
      GPIOE_PCOR |= datadirectionregister; // set outputs low
      if (datadirectionregister & (1<<0))
        pinMode(31, OUTPUT);	// strip #1
      if (datadirectionregister & (1<<1))
        pinMode(26, OUTPUT);	// strip #2
      break;
    default: // default == D
      GPIOD_PCOR |= datadirectionregister; // set outputs low
      if (datadirectionregister & (1<<0))
        pinMode(2, OUTPUT);  // strip #1
      if (datadirectionregister & (1<<1))
        pinMode(14, OUTPUT); // strip #2
      if (datadirectionregister & (1<<2))
        pinMode(7, OUTPUT);  // strip #3
      if (datadirectionregister & (1<<3))
        pinMode(8, OUTPUT);  // strip #4
      if (datadirectionregister & (1<<4))
        pinMode(6, OUTPUT);  // strip #5
      if (datadirectionregister & (1<<5))
        pinMode(20, OUTPUT); // strip #6
      if (datadirectionregister & (1<<6))
        pinMode(21, OUTPUT); // strip #7
      if (datadirectionregister & (1<<7))
        pinMode(5, OUTPUT);  // strip #8
      break;
  }

	// create the two waveforms for WS2811 low and high bits
	frequency = (params & WS2811_400kHz) ? 400000 : 740000;
  setFTM_Timer(WS2811_TIMING_T0H, WS2811_TIMING_T1H, frequency);
	// analogWriteResolution(8);
	// analogWriteFrequency(3, frequency);
	// analogWriteFrequency(4, frequency);
	// analogWrite(3, WS2811_TIMING_T0H);
	// analogWrite(4, WS2811_TIMING_T1H);

	// // pin 16 triggers DMA(port B) on rising edge (configure for pin 3's waveform)
	// CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);
	// pinMode(3, INPUT_PULLUP); // pin 3 no longer needed

	// // pin 15 triggers DMA(port C) on falling edge of low duty waveform
	// // pin 15 and 16 must be connected by the user: 16 is output, 15 is input
	// pinMode(15, INPUT);
	// CORE_PIN15_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(1);

	// // pin 4 triggers DMA(port A) on falling edge of high duty waveform
	// CORE_PIN4_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(3);

	// DMA channel #1 sets WS2811 high at the beginning of each cycle
	dma1.TCD->SADDR = &ones;
	dma1.TCD->SOFF = 0;
	dma1.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma1.TCD->NBYTES_MLNO = 1;
	dma1.TCD->SLAST = 0;
  switch(dataport)
  {
    case 'E':
      dma1.TCD->DADDR = &GPIOE_PSOR;
      break;
    default: // default == D
      dma1.TCD->DADDR = &GPIOD_PSOR;
      break;
  }
	dma1.TCD->DOFF = 0;
	dma1.TCD->CITER_ELINKNO = bufsize;
	dma1.TCD->DLASTSGA = 0;
	dma1.TCD->CSR = DMA_TCD_CSR_DREQ;
	dma1.TCD->BITER_ELINKNO = bufsize;

	// DMA channel #2 writes the pixel data at 20% of the cycle
	dma2.TCD->SADDR = frameBuffer;
	dma2.TCD->SOFF = 1;
	dma2.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma2.TCD->NBYTES_MLNO = 1;
	dma2.TCD->SLAST = -bufsize;
  switch(dataport)
  {
    case 'E':
      dma2.TCD->DADDR = &GPIOE_PDOR;
      break;
    default: // default == D
      dma2.TCD->DADDR = &GPIOD_PDOR;
      break;
  }
	dma2.TCD->DOFF = 0;
	dma2.TCD->CITER_ELINKNO = bufsize;
	dma2.TCD->DLASTSGA = 0;
	dma2.TCD->CSR = DMA_TCD_CSR_DREQ;
	dma2.TCD->BITER_ELINKNO = bufsize;

	// DMA channel #3 clear all the pins low at 48% of the cycle
	dma3.TCD->SADDR = &ones;
	dma3.TCD->SOFF = 0;
	dma3.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma3.TCD->NBYTES_MLNO = 1;
	dma3.TCD->SLAST = 0;
  switch(dataport)
  {
    case 'E':
      dma3.TCD->DADDR = &GPIOE_PCOR;
      break;
    default: // default == D
      dma3.TCD->DADDR = &GPIOD_PCOR;
      break;
  }
	dma3.TCD->DOFF = 0;
	dma3.TCD->CITER_ELINKNO = bufsize;
	dma3.TCD->DLASTSGA = 0;
	dma3.TCD->CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
	dma3.TCD->BITER_ELINKNO = bufsize;

#ifdef __MK20DX256__
	MCM_CR = MCM_CR_SRAMLAP(1) | MCM_CR_SRAMUAP(0);
	AXBS_PRS0 = 0x1032;
#endif

	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH0);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH1);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH2);

	// enable a done interrupts when channel #3 completes
	dma3.attachInterrupt(isr);
	//pinMode(1, OUTPUT); // testing: oscilloscope trigger
}

void OctoWS2811::isr(void)
{
	
	update_completed_at = micros();
  dma3.clearInterrupt();
	update_in_progress = 0;
}

int OctoWS2811::busy(void)
{
	//if (DMA_ERQ & 0xE) return 1;
	if (update_in_progress) return 1;
	// busy for 50 us after the done interrupt, for WS2811 reset
	if (micros() - update_completed_at < 50) return 1;
	return 0;
}

void OctoWS2811::show(void)
{
	uint32_t sc;
	// wait for any prior DMA operation
	while (update_in_progress) ; 
	// it's ok to copy the drawing buffer to the frame buffer
	// during the 50us WS2811 reset time
	if (drawBuffer != frameBuffer) {
		// TODO: this could be faster with DMA, especially if the
		// buffers are 32 bit aligned... but does it matter?
		memcpy(frameBuffer, drawBuffer, stripLen * 24);
	}

	// ok to start, but we must be very careful to begin
	// without any prior 3 x 800kHz DMA requests pending
	sc = FTM0_SC;
	//digitalWriteFast(1, HIGH); // oscilloscope trigger
  
  //noInterrupts(); // This code is not time critical anymore. IRQ can stay on. 
  // We disable the FTM Timer, reset it to its initial counter value and clears all irq-flags. 
  // Clearing irqs is a bit tricky, because with DMA enabled, only the DMA can clear them. 
  // We have to disable DMA, reset the irq-flags and enable DMA once again.
	update_in_progress = 1;
  FTM0_SC = sc & 0xE7;	// stop FTM1 timer

  FTM0_CNT = 0; // writing any value to CNT-register will load the CNTIN value!
  
  FTM0_C0SC = 0; // disable DMA transfer. It has to be done, because we can't reset the CHnF bit while DMA is enabled
  FTM0_C1SC = 0;
  FTM0_C2SC = 0;
  
  FTM0_STATUS; // read status and write 0x00 to it, clears all IRQs
  FTM0_STATUS = 0x00;

  FTM0_C0SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28; 
  FTM0_C1SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28;
  FTM0_C2SC = FTM_CSC_DMA | FTM_CSC_CHIE | 0x28;

	dma1.enable();
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	//interrupts();
	//digitalWriteFast(1, LOW);
	// wait for WS2811 reset
	while (micros() - update_completed_at < 50) ;
	FTM0_SC = sc;		// restart FTM timer
}

void OctoWS2811::clear()
{
  uint32_t bufsize = stripLen*24;
	memset(frameBuffer, 0, bufsize);
	if (drawBuffer) {
		memset(drawBuffer, 0, bufsize);
	} else {
		drawBuffer = frameBuffer;
	}
}
void OctoWS2811::white()
{
  uint32_t bufsize = stripLen*24;
	memset(frameBuffer, 255, bufsize);
	if (drawBuffer) {
		memset(drawBuffer, 255, bufsize);
	} else {
		drawBuffer = frameBuffer;
	}
}
  
void OctoWS2811::setPixel(uint32_t num, int color)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	
	switch (params & 7) {
	  case WS2811_RBG:
		color = (color&0xFF0000) | ((color<<8)&0x00FF00) | ((color>>8)&0x0000FF);
		break;
	  case WS2811_GRB:
		color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);
		break;
	  case WS2811_GBR:
		color = ((color<<8)&0xFFFF00) | ((color>>16)&0x0000FF);
		break;
	  default:
		break;
	}
	strip = num / stripLen;  // Cortex-M4 has 2 cycle unsigned divide :-)
	offset = num % stripLen;
	bit = (1<<strip);
	p = ((uint8_t *)drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {
		if (color & mask) {
			*p++ |= bit;
		} else {
			*p++ &= ~bit;
		}
	}
}

int OctoWS2811::getPixel(uint32_t num)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	int color=0;

	strip = num / stripLen;
	offset = num % stripLen;
	bit = (1<<strip);
	p = ((uint8_t *)drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {
		if (*p++ & bit) color |= mask;
	}
	switch (params & 7) {
	  case WS2811_RBG:
		color = (color&0xFF0000) | ((color<<8)&0x00FF00) | ((color>>8)&0x0000FF);
		break;
	  case WS2811_GRB:
		color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);
		break;
	  case WS2811_GBR:
		color = ((color<<8)&0xFFFF00) | ((color>>16)&0x0000FF);
		break;
	  default:
		break;
	}
	return color;
}
