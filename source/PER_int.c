/****************************************************************
* FILENAME:     PER_int.c
* DESCRIPTION:  periodic interrupt code
* AUTHOR:       Mitja Nemec
* DATE:         16.1.2009
*
****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"

// za meritev toka
long    tok_raw = 0;
long    tok_offset = 1940;
float   tok_gain = (3.3 / (4096 * 5.1 * 50));
float   tok = 0.0;

// vklopno razmerje
float   duty = 0.0;

// uporabljeni spremenljivke

float error=0.0;//trenutna napaka,pogrešek
float error_total=0.0;//akumilirana napaka
float last_error=0.0; //zadnja napaka
float output=0.0; //iz regulatorja
int negative_speed_flag=0.0;//zazna negativno vrednost napetosti


float K_p=0.723; //proporcionalno ojaèenja
float K_i=0.000207;//integrator-ojaèenja
//float K_d=0.000023;
float K_d=10000000000;
float razlika;

float   P_clen=0.0;
float   I_clen=0.0;
float   D_clen=0.0;

// za kalibracijo preostale napetosti tokovne sonde
bool    offset_calib = TRUE;
long    offset_counter = 0;
float   offset_filter = 0.005;

// za oceno obremenjenosti CPU-ja
float   cpu_load  = 0.0;
long    interrupt_cycles = 0;

// za stopnico vklopnega razmerja
float   ref_counter = 0;
float   ref_counter_prd = 5L * SAMPLE_FREQ;
float   ref_counter_cmpr = 5L * SAMPLE_FREQ/2;
float   ref_value_low = 0.0;
float   ref_value_high = 0.5; //-0.5 obrnemo smer zeljene vrednosti
float   ref_value = 0.0;


// spremenljikva s katero štejemo kolikokrat se je prekinitev predolgo izvajala
int     interrupt_overflow_counter = 0;

/**************************************************************
* spremenljivke, ki jih potrebujemo za meritev hitrosti
**************************************************************/
float kot_iz_senzorja = 0.0;
float kot = 0.0;
float hitrost = 0.0;

// obstojeèa meritev
float hitrost_narejena = 0.0;
extern float SPD_measure(float kot_iz_senzorja);

/**************************************************************
* spremenljivke, ki jih potrebujemo za regulacijo hitrosti
**************************************************************/
float zeljena = 0.0;

/**************************************************************
* spremenljivke, ki jih potrebujemo za alfa beta filter
**************************************************************/
float hitrost_abf = 0.0;
/**************************************************************
* Prekinitev, ki v kateri se izvaja regulacija
**************************************************************/
#pragma CODE_SECTION(PER_int, "ramfuncs");
void interrupt PER_int(void)
{
    /* lokalne spremenljivke */


//	float new_speed;
    // najprej povem da sem se odzzval na prekinitev
    // Spustimo INT zastavico casovnika ePWM1
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Spustimo INT zastavico v PIE enoti
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    
    // pozenem stoprico
    interrupt_cycles = TIC_time;
    TIC_start();

    // izracunam obremenjenost procesorja
    cpu_load = (float)interrupt_cycles / (CPU_FREQ/SAMPLE_FREQ);

    // pocakam da ADC konca s pretvorbo
    ADC_wait();

    // ali kalibriram preostalo napetost tokovne sonde
    if (offset_calib == TRUE)
    {
        tok_raw = TOK;
        tok_offset = (1.0 - offset_filter) * tok_offset + offset_filter * tok_raw;

        offset_counter = offset_counter + 1;
        if ( offset_counter == SAMPLE_FREQ)
        {
            offset_calib = FALSE;
        }

    }
    // sicer pa normalno obratujem
    else
    {
        // preracunam napetost
        tok_raw = TOK - tok_offset;
        tok = -tok_raw * tok_gain;

        // generiram spremenljivo vklopno razmerje
        ref_counter = ref_counter + 1;
        if (ref_counter >= ref_counter_prd)
        {
            ref_counter = 0;
        }

        // stopnicasta zeljena vrednost
        if (ref_counter > ref_counter_cmpr)
        {
            ref_value = ref_value_low;
        }
        else
        {
            ref_value = ref_value_high;
        }
        
        /*******************************************************
        * Tukaj pride koda izracuna hitrosti
        *******************************************************/
        // preberem kot iz senzorja
        kot_iz_senzorja = SPI_getkot();

        // izracunam hitrost
        hitrost_narejena = SPD_measure(kot_iz_senzorja);
        
        
        /*******************************************************
        * Tukaj pride koda za alfa beta filter
        *******************************************************/
        

        /*******************************************************
        * Tukaj pride koda regulatorja hitrosti
        *******************************************************/
        zeljena = ref_value;

        if (zeljena<0) //preverjanje smeri zeljene vrednosti hitrosti
        {
        	negative_speed_flag=1;
        }
        else
        {
          	negative_speed_flag=0;
        }

        if(zeljena==0.0)
        {
        error=0.0; //vsak cikel ponovno resetiraj napako
        output=0.0;
        error_total=0.0;  //resetiraj akumulirano napako, ko je zeljena vrednost 0
        }

        if((zeljena!=0)&&(negative_speed_flag==1))
        {
        error=zeljena*(-1)-hitrost_narejena*(-1); //izraèuna trenutno napako
        error_total+=error; //napako prištej skupni napaki
        }
        else
        {
        error=zeljena-hitrost_narejena; //izraèuna trenutno napako
        error_total+=error; //napako prištej skupni napaki
        }

         //zadnja napaka
        //regulacijski èleni
        P_clen=error*K_p;
        I_clen=error_total*K_i;
        razlika=error - last_error;
        D_clen=K_d*(error - last_error);
        last_error=error;

        output=P_clen+I_clen+D_clen; //izhod iz regulatorja

        if(zeljena==0.0) //v mirovanju ponovno resetiraj vrednosti
             {
             error=0.0; //vsak cikel ponovno resetiraj napako
             output=0.0;
             error_total=0.0;  //resetiraj akumulirano napako, ko je zeljena vrednost 0
             }

        if(output < 0.0) //duty omejimo med 0 in 1
        	{
        	duty=0.0;
        	}
        else
        	{
        	if(output >=1.0)
        		{
        		if(negative_speed_flag==1) // omeji duty cycle glede na predznak hitrosti
        			{
        	     duty=(-1.0);
        			}
        		else
        			{
        			duty=1.0;
        			}
        		}
        	else
        		{
        			if(negative_speed_flag==1) // omeji duty cycle glede na predznak hitrosti
        			{
        				duty=output*(-1);
        			}
        			else
        			{
        				duty=output;
         			}
        		}
        	 }

        // meritev hitrosti
        //duty = ref_value;
        // osvežim vklopno razmerje
        PWM_update(duty);
        
        // spavim vrednosti v buffer za prikaz
        DLOG_GEN_update();
    }

    
    /* preverim, èe me sluèajno èaka nova prekinitev.
       èe je temu tako, potem je nekaj hudo narobe
       saj je èas izvajanja prekinitve predolg
       vse skupaj se mora zgoditi najmanj 10krat,
       da reèemo da je to res problem
    */
    if (EPwm1Regs.ETFLG.bit.INT == TRUE)
    {
        // povecam stevec, ki steje take dogodke
        interrupt_overflow_counter = interrupt_overflow_counter + 1;
        
        // in ce se je vse skupaj zgodilo 10 krat se ustavim
        // v kolikor uC krmili kakšen resen HW, potem moèno
        // proporoèam lepše "hendlanje" takega dogodka
        // beri:ugasni moènostno stopnjo, ...
        if (interrupt_overflow_counter >= 10)
        {
            asm(" ESTOP0");
        }
    }
    
    // stopam
    TIC_stop();

}   // end of PWM_int

/**************************************************************
* Funckija, ki pripravi vse potrebno za izvajanje
* prekinitvene rutine
**************************************************************/
void PER_int_setup(void)
{

    // Proženje prekinitve 
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    //sproži prekinitev na periodo
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;         //ob vsakem prvem dogodku
    EPwm1Regs.ETCLR.bit.INT = 1;                //clear possible flag
    EPwm1Regs.ETSEL.bit.INTEN = 1;              //enable interrupt

    // inicializiram data logger
    dlog.trig_value = ref_counter_prd - 1000;   // specify trigger value
    dlog.slope = Positive;                      // trigger on positive slope
    dlog.prescalar = 100;                       // store every  sample
    dlog.mode = Normal;                         // Normal trigger mode
    dlog.auto_time = 100;                       // number of calls to update function
    dlog.holdoff_time = 100;                    // number of calls to update function

    dlog.trig = &ref_counter;
    dlog.iptr1 = &kot_iz_senzorja;
    dlog.iptr2 = &hitrost_narejena;
    dlog.iptr3 = &hitrost;
    //dlog.iptr4 = &hitrost_abf;
    dlog.iptr4 = &razlika;


    // registriram prekinitveno rutino
    EALLOW;
    PieVectTable.EPWM1_INT = &PER_int;
    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    IER |= M_INT3;
    // da mi prekinitev teèe  tudi v real time naèinu
    // (za razhoršèevanje main zanke in BACK_loop zanke)
    SetDBGIER(M_INT3);
}
