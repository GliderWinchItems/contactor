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
	p->size     = 24;
	p->crc      = 0;
   p->version  = 1;

	p->hbct     = 1000;  // Time (ms) between HB msg

	// Vrefint calibration
	p->vdd      = 3.005;	// Vdd for following vrefave
	p->vrefave  = 1488.0;// ADC reading for above Vdd

	// Internal temperature sensor calibration
	p->v25      = 1710;  // 25 deg C ADC reading
	p->slope    = 4.3;   // mv/degC slope of temperature sensor

	// ADC input calibrations
	p->cal_cur1.offset    = 0.0;
   p->cal_cur1.scale     = 1.4494E-4;
	p->cal_cur1.iir.k     = 10; // Filter time constant
	p->cal_cur1.iir.scale = 2;  // Integer scaling

	p->cal_cur2.offset    = 0.0;
   p->cal_cur2.scale     = 1.4494E-2;
	p->cal_cur2.iir.k     = 10;
	p->cal_cur2.iir.scale = 2;

	p->cal_5v.offset    = 0.0;
   p->cal_5v.scale     = 0.0014494022;
	p->cal_5v.iir.k     = 10;
	p->cal_5v.iir.scale = 2;

	p->cal_12v.offset   = 0.0;
	p->cal_12v.scale    = 0.004891904;
	p->cal_12v.iir.k     = 10;
	p->cal_12v.iir.scale = 2;



	return 0;	
}
