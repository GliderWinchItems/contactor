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

	// Vrefint calibration
	p->vdd      = 3.005;	     // Vdd for following vrefave
	p->vrefave  = (16*1488.0);// ADC reading (DMA sum) for above Vdd

	// Internal temperature sensor calibration
	p->v25      = (16*1710);  // 25 deg C ADC (DMA sum) reading
	p->slope    = 4.3;        // mv/degC slope of temperature sensor

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
