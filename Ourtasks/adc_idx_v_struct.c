/******************************************************************************
* File Name          : adc_idx_v_struct.c
* Date First Issued  : 06/17/2019
* Board              :
* Description        : Translate parameter index into pointer into struct
*******************************************************************************/

/* **************************************************************************************
 * int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * ************************************************************************************** */
int adc_idx_v_struct_hardcode_params(struct ADCCONTACTORLC* p)
{
/* Copy for convenience--
 struct ADCCONTACTORLC
 {
	uint32_t size;			// Number of items in struct
 	uint32_t crc;			// crc-32 placed by loader
	uint32_t version;		// struct version number
	uint32_t hbct;			// Heartbeat ct: ticks between sending msgs
	float vdd;           // Vrefint calibration: Vdd voltage for vrefave reading
	float vrefave;       // Vrefint calibration: Average vref ADC readings at Vdd
	struct ADCCALCONTACTOR cal_cur1; // Current calibration (offset & scale)
	struct ADCCALCONTACTOR cal_cur2; // Current calibration (offset & scale)
	struct ADCCALCONTACTOR cal_5v;   // 5v regulated voltage 
	struct ADCCALCONTACTOR cal_12v;  // 12v raw CAN voltage
 };
*/
	p->size     = 24; // Number of items in list
	p->crc      = 0;  // Do later
   p->version  = 1;

	p->hbct     = 1000;  // Time (ms) between HB msg

/* Reproduced for convenience 
struct ADC1CALINTERNAL
{
	struct IIR_L_PARAM iiradcvref; // Filter: adc readings: Vref 
	struct IIR_L_PARAM iiradctemp; // Filter: adc readings: temperature
	uint32_t adcvdd;   // (ADC reading) for calibrating Vdd (3.3v)
	uint32_t adcrmtmp; // (ADC reading) room temperature reading
	uint32_t rmtmp;    // Room temp for reading (deg C)
	double dvdd;       // (double) measured Vdd (volts)
	double dslope;     // (double) mv/degC temperature sensor slope
};
*/
	p->calintern.iiradcvref.k     = 10;    // Filter time constant
	p->calintern.iiradcvref.scale = 2;

	p->calintern.iiradctemp.k     = 10;    // Filter time constant
	p->calintern.iiradctemp.scale = 2;

	p->calintern.dvdd   = 3.005;	    // Vdd for following vrefave
	p->calintern.adcvdd = (16*1488.0);// ADC reading (DMA sum) for above Vdd

	// Internal temperature sensor calibration
	p->adcvrmtmp = (16*1710);  // Room temp ADC (DMA sum) reading
	p->rmtmp     = 27;         // Room temp for ADC reading     
	p->dslope    = 4.3;        // mv/degC slope of temperature sensor

/*  Reproduced for convenience 
struct ADCCALHE
{
	struct IIR_L_PARAM iir; // Filter: Time constant, integer scaling
	double   scale;     // Resistor ratio to scale to desired units
	uint32_t j5adcve;   // jumpered to 5v: adc reading HE input
	uint32_t j5adcv5;   // jumpered to 5v: adc reading 5v input
	uint32_t zeroadcve; // connected, no current: HE adc reading
	uint32_t zeroadc5;  // connected, no current: 5v adc reading 
	uint32_t caladcve;  // connected, cal current: adc reading
	 int32_t calcur;    // connected, cal current: current
};
*/
	p->cal_cur1.iir.k     = 10;    // Filter time constant
	p->cal_cur1.iir.scale = 2;




	// ADC input calibrations
	p->cal_cur1.cald.offset    = -0.5;  // 1/2 Vdd
   p->cal_cur1.cald.scale     = 1.4494E-4; // Vmeasured/ADCdmasumreading
	p->cal_cur1.cali.iir.k     = 10;    // Filter time constant
	p->cal_cur1.cald.iir.scale = 2;     // Integer scaling

	p->cal_cur2.cald.offset    = -0.5;  // 1/2 Vdd
   p->cal_cur2.cald.scale     = 1.4494E-2;
	p->cal_cur2.cali.iir.k     = 10;
	p->cal_cur2.cali.iir.scale = 2;

	p->cal_5v.cald.offset      = 0.0;
   p->cal_5v.cald.scale       = 0.0014494022;
	p->cal_5v.cali.iir.k       = 10;
	p->cal_5v.cali.iir.scale   = 2;

	p->cal_12v.cald.offset     = 0.0;
	p->cal_12v.cald.scale      = 0.004891904;
	p->cal_12v.cali.iir.k      = 10;
	p->cal_12v.cali.iir.scale  = 2;

	return 0;	
}
