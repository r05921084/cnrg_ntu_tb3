/* audimod.c */

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

/***************************************************************************
    A U D I T O R Y  M O D E L  B A S E D  S P E E C H  A N A L Y S I S
 ***************************************************************************/

#include "audiprog.h"
#include "audimod.h"
#include "decimation.h"
#include "filterbank.h"
#include "hcmbank.h"


/* KT 19990525
#include "ecebank.h"
#include "cpu.h"
*/

#define width 16

static double delay; /* delay introduced by model    */
static parameters par[width - 1 + 1];
static double tend, tout;
static double t, Tsmp;
static long pitch_delay; /* get pitch from frame[n+pitch_delay] */

void setup_modules()
/***********************************************************************
  Add extra 10 ms to pitch delay, due to pitch window length which
  is larger than erl-analysis window length
 ***********************************************************************/
{
  startup_sigio();
  setup_omef();
  setup_filterbank();
  setup_decimation(); /* KT: JPM "setup_decimation NA setup_filterbank" */
  setup_hcmbank();

  /* KT 19990525
 setup_ecebank(); 
 setup_cpu();

 pitch_delay=shift;
 if (pitch_delay>7) printf("%s\n","Error: Tframe must be > 3 ms (for pitch)");
*/
}

void init_modules(const char *inOutputFileName)
{
  init_omef();
  init_decimation();
  init_filterbank();
  init_hcmbank(inOutputFileName);
}

/* Finalize modules */
/* KT 19990525      */
void finish_modules()
{
  finish_hcmbank();
}

long specify_parameters(long inNumOfChannels, double inFirstFreq, double inFreqDist, double inSampleFrequency)
{
  /* KT adapted */
  if (inNumOfChannels > max_nchan)
  {
    printf("ERROR:\nToo many channels! Max. = 40");
    return -1;
  }
  nchan = inNumOfChannels;          // given, checked with max_nchan
  uc1 = inFirstFreq;                // given
  duc = inFreqDist;                 // given
  fssig = inSampleFrequency / 1000; // (kHz) should better be extracted from sound file...
  
  ndecim = 1;
  Tse = 2.0 / fssig;

  Terl = Tframe / Nerl;
  // Tframe stays fixed to 10 (time between frames)
  // Nerl stays fixed to 5 (number of erl samples/frame)
  return 0;
}

long startup_audiprog(long inNumOfChannels, double inFirstFreq, double inFreqDist, double inSampleFrequency)
{
  long theResult = 0;
  theResult = specify_parameters(inNumOfChannels, inFirstFreq, inFreqDist, inSampleFrequency);
  if (theResult == 0)
  {
    setup_modules();
    delay = Tdecim + Tmodel;
    printf("%s%7.3f%7.3f%7.3f%4d\n", "Td,Tm,delay,Ne =", Tdecim, Tmodel, delay, Ne);
    nspect = nchan;
    nr_of_par = nchan + Nerl + 3;
    return 0;
  }
  else return theResult;
}

int init_analysis(const char *inOutputFileName)
/**********************************************************************
    The signal is supposed to be surrounded by two silent intervals
    of at least 20 ms long.
    The spectrum generated at frame n is supposed to correspond to
    time n.Tframe (n=1..nframe)
 **********************************************************************/
{
  int m, p;

  init_modules(inOutputFileName);
  n = 0;
  nmod = 0;
  t = 0;
  tout = delay + Tframe;
  tend = 0;
  Tsmp = 1 / fsmp;
  for (m = 0; m < width; m++)
    for (p = 1; p < nchan + Nerl + 4; p++)
      par[m][p] = 0;
  return 0;
}

/* Finalize analysis of one file */
/* KT 19990525 */
void finish_analysis()
{
  finish_modules();
}

int one_frame(int *last, parameters frame)
{
  double sn;
  
  if (tend)  // final padding EDWARDCHEN
    sn = 0;
  else
  {
    sn = factor * new_sample(last);
    if (*last)
    {
      tend = n * Tsmp + delay + 2 * Tframe;
      // tend = n * Tsmp + delay;
    }
  }

  sn = omef(sn);
  decimate(sn);
  filterbank();
  hcmbank();

  if (fsmp != fssig)
  {
    sn = 0;
    n++;
    t = t + Tsmp;
    if (++nmod == max_step)
      nmod = 0;
    decimate(sn);
    filterbank();
    hcmbank();
  }
  
  n++;
  t = t + Tsmp;
  if (++nmod == max_step)
    nmod = 0;

  if (((n & Nemask) == 0) && (t >= tout))
  {
    tout = tout + Tframe;
  }

  if (tend)
  {
    if(tout < tend)
      *last = 0;
    else
      *last = 1;
  }

  // if ((tend != 0) && (tout >= tend))
  //   {
  //     printf("if ((tend != 0) && (tout >= tend))\n");
  //     // *last = 1;
  //   }
  // else
  //   {
  //     if ((tend != 0) && (tout < tend))
  //       printf("if ((tend != 0) && (tout < tend)) %f, %f, %f\n", tend, tout, sn);
  //     else
  //       printf("if ((tend == 0)\n");
  //     if (*last)
  //       printf("write *last = 0 back.\n");
  //     *last = 0;
  //   }

  return 0;
}
