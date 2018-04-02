/* audiprog.c */

/*------------------------------------------------------------------------------
    IPEM Toolbox - Toolbox for perception-based music analysis 
    Copyright (C) 2005 Ghent University 
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
------------------------------------------------------------------------------*/

/* adapted KT 19990802 */
/* Processes one sound file  */

/***************************************************************************
    A U D I T O R Y  M O D E L  B A S E D  S P E E C H  A N A L Y S I S
 ***************************************************************************/

#include <filenames.h>
#include "audiprog.h"
#include "audimod.h"

static text_line infile;
static int w, w1, w2;

static text_line s;

double Tdecim;		 /* delay introduced by decimation unit (ms)  */
double Tmodel;		 /* delay introduced by rest of model (ms)    */
double fsmp = 20.0;  /* internal sampling frequency = 1/Tsmp      */
double fssig = 10.0; /* signal sampling frequency                 */
double Tframe = 10;  /* time between successive frames (ms)       */
int ndecim;			 /* number of decimation filters to use       */
double Tse;			 /* time between successive envelope samples  */
int Ne;				 /* Tsmp for envelope / Tsmp of model         */
int Nemask;			 /* Ne-1 = mask for MOD replacement           */
int Nerl = 5;		 /* number of erl samples per frame           */
double Terl;		 /* time between erl samples in frame         */
int shift;			 /* pitch comes from SHIFT frames behind      */
int nchan = 20;		 /* number of filterbank channels             */
double uc1 = 2.0;	/* ucp of first channel                      */
double duc = 0.85;   /* spacing between succesive ucp's           */
int n;				 /* time index                                */

rvector fc;		  /* BPF central frequencies                   */
rvector uc;		  /* corresponding critical band units         */
ivector x2;		  /* is there a need to upsample after BPF?    */
ivector step;	 /* time steps used in analysis channels      */
ivector stepmask; /* stepmask=step-1 = mask for MOD replacement*/
/* JPM: 20/10/98: new implementation of channel selection ********/
int max_step;   /* maximum step encountered in channels      */
int nmod;		/* time modulo max_step                      */
int low_ch[16]; /* lowest channel to process for different   */
				/* values of nmod                            */
/*****************************************************************/
double decim[5 + 1]; /* decimation products                       */
ivector indx;		 /* index in decimation product array         */
rvector ybpf;		 /* BPF outputs at multiples of step.Tsmp     */

rvector yhcm;	 /* HCM outputs at multiples of Tse           */
rvector yhcm1;	/* previous HCM outputs at multiples of Tse  */
rvector ev, erl;  /* virtual tone, roughness+loudness comps.   */
rvector prev_erl; /* previous roughness+loudness components    */
rvector yres;	 /* xxx outputs at multiples of step.Tsmp     */
double factor = 1.0;	/* multiplication factor for input samples   */

long analyse_signal(const char *inOutputFile)
/**********************************************************************
    The signal is supposed to be surrounded by two silent intervals 
    of at least 20 ms long.
    The spectrum generated at frame n is supposed to correspond to
    time n.Tframe (n=1..nframe).
 **********************************************************************/
{
	int last;
	parameters frame;

	if (init_analysis(infile, inOutputFile))
		return -1;

	printf("Analysing %s\n", infile);
	do
	{
		one_frame(&last, frame);
	} while (!last);

	printf("nsamp: %d\n", n);
	finish_analysis(); /* KT 19990525 */

	return 0;
}

// -----------------------------------------------------------------------------
//  AudiProg
// -----------------------------------------------------------------------------
// Main entry point for the auditory model

long AudiProg(long inNumOfChannels, double inFirstFreq, double inFreqDist,
			  const char *inInputFileName, const char *inInputFilePath,
			  const char *inOutputFileName, const char *inOutputFilePath,
			  double inSampleFrequency, long inSoundFileFormat)
{
	long theResult = 0;
	char theOutputFile[256];

	if (inSoundFileFormat < 2){
		printf("No longer support format code < 2!!! EDWARDCHEN");
		return -1;
	}

	set_sigioread_format(inSoundFileFormat);

	// Setup input file
	if (strlen(inInputFilePath) == 0)
		infile[0] = '\0';
	else
	{
		strcpy(infile, inInputFilePath);
		strcat(infile, "/");
	}
	strcat(infile, inInputFileName);

	// Setup output file
	if (strlen(inOutputFilePath) == 0)
		theOutputFile[0] = '\0';
	else
	{
		strcpy(theOutputFile, inOutputFilePath);
		strcat(theOutputFile, "/");
	}
	strcat(theOutputFile, inOutputFileName);

	theResult = startup_audiprog(inNumOfChannels, inFirstFreq, inFreqDist, inSampleFrequency);

	if (theResult == 0)
		return analyse_signal(theOutputFile);		
	else
		return theResult;
		
	
}
