/**************************************************************
 *	satnav.c					      *
 *							      *
 *	Software for position fixing using the U S	      *
 *	Navy Transit Satellite navigation system.	      *
 *							      *
 *	1988/89 Project 47, R S Barker and R J Finean	      *
 **************************************************************/

#include <stdio.h>
#include <dos.h>
#include <time.h>

/* Macros */

#define max(x, y)	((x) > (y) ? (x) : (y))
					/* Max val of x and y */
#define min(x, y)	((x) < (y) ? (x) : (y))
					/* Min val of x and y */
#define sqr(x)		((x) * (x))	/* Square of x */
#define cube(x) 	((x) * (x) * (x))
					/* Cube of x */

/* Constants */

#define TRUE		1
#define FALSE		0
#define OK		TRUE
#define FAIL		FALSE
#define NOLIMIT 	-1
#define BUFFSIZE	512		/* Input buffer size */
#define PORT		0x300		/* Input port number */
#define IRQNO		3		/* H/W Interrupt no. */
#define SIRQNO		(IRQNO + 8)	/* S/W interrupt no. */
#define DISIRQ		(1<<IRQNO)	/* Disable ints mask */
#define ENIRQ		(0xFF ^ DISIRQ) /* Enable ints mask */
#define IRQCTRL 	0x20		/* Interrupt OCW2 */
#define IMR		0x21		/* Int. mask register */
#define CLEAR		0x20		/* Non-specific EOI */
#define NOLOCK		2		/* No PLL lock error */
#define DOUBERR 	4		/* Doublet sync error */
#define LOST		8		/* Buffer full error */
#define RXERR		1E+13		/* Receive error */
#define MAXPTS		10		/* Max no of data pts */
#define SYNC0		0x5556		/* Sync word part 1 */
#define SYNC1		0x5555		/* Sync word part 2 */
#define SYNC2		0x5555		/* Sync word part 3 */
#define SYNC3		0x0006		/* Sync word part 4 */
#define FDSS		(sizeof(struct fdat) / sizeof(double))
					/* Fixed data size */
#define VINT		(7*sizeof(struct vdat)/sizeof(double))
					/* Var data interval */
#define HOURS24 	(24 * 60) 	/* 24 hours in mins */
#define HOURS8		(8 * 60)	/* 8 hours in minutes */
#define PI		3.1415926536
#define GEOIDHT 	0.065		/* Sea level above
							geoid */
#define SEAHT		0.1		/* Antenna height
					      above sea level */
#define MAXITS		1000		/* Max iterations for
					      position fixing */
#define LATTOL		1E-3		/* Latitude tolerance */
#define LNGTOL		1E-3		/* Longitude tol */
#define FTOL		1000		/* Offset freq tol */
#define WEARTH		4.3752695E-3	/* Rotation of Earth
						 (rads / min) */
#define REARTH		6378.166	/* Equatorial radius
							 (km) */
#define FLAT		3.35232E-3	/* Flattening of
						     spheroid */
#define INTFREQ		(532E3 * 60) 	/* Nominal final IF
					       (cycles / min) */
#define L0		7.48545463	/* Vacuum wavelength of
					nominal receiver freq */
#define ESTLAT		53.809		/* Leeds Uni Elec Eng */
#define ESTLNG		-1.555		/* estimated position */

int nkdec[] = {100, 140, 130, 120, 110, 0,  40,  30,  20,  10};
					/* nk digit 1 lookup */

/* Data structures */

struct fdat			/* Fixed parameter data */
		     /* All elements must be of type 'double' */
	    {
	      double tp;	/* Time of 1st perigee (mins) */
	      double n; 	/* Mean motion (rads / min) */
	      double w; 	/* Argument of perigee (rads) */
	      double wdot;	/* Precession rate of w
						(rads / min) */
	      double e0;	/* Eccentricity */
	      double a0;	/* Mean semi-major axis (km) */
	      double omega;	/* Right ascension (rads) */
	      double omegadot;	/* Precession rate of node
						(rads / min) */
	      double ci;	/* Cosine of inclination */
	      double lambdag;	/* Inertial longitude of
					Greenwich (rads) */
	      double id;	/* Satellite I D number */
	      double si;	/* Sine of inclination */
	    };

struct vdat			/* Variable parameter data */
		     /* All elements must be of type 'double' */
	    {
	      double tk;	/* Fiducial time (mins) */
	      double deltaek;	/* Eccentric anomaly correction
						(rads) */
	      double deltaak;	/* Semi-major axis correction
						(kilometers) */
	      double nk;	/* Out of plane orbit component
						(kilometers) */
	      double dc;	/* Doppler count */
	    };

struct mess			/* Message data */
	    {
	      unsigned long dopcount;
				/* Raw doppler count */
	      time_t rxtime;	/* Receive time of message */
	    };

struct coords			/* x, y, z coordinates */
	      {
		double x;
		double y;
		double z;
	      };

struct partial			/* Partial derivatives */
	      {
		double bylat;	/* with respect to latitude */
		double bylng;	/* with respect to longitude */
	      };

/* Declare functions returning non-integers */

extern void interrupt inface();
extern unsigned long getdop();
extern double verdict();
extern void storwrd(), satpoints(), navpoints(), slants();

/* Assign external variable workspace */

int buffer[BUFFSIZE];		/* Input buffer */
int *cwm;			/* Current write marker */
int *crm;			/* Current read marker */
double lat, lng;		/* Latitude & Longitude */
struct fdat fixdata;		/* Fixed parameters structure */
struct vdat vardata[MAXPTS];	/* Variable data struct array */

/**************************************************************/

void main	/* Sets up interrupt vectors and controls */
   (void)	/* sequence of collecting data and	  */
		/* calculating position. Resets interrupt */
		/* vector on exit.			  */

		/* 17/1/89, 14/3/89 R J Finean */

{
  int finished = FALSE; 	/* Exit program flag */
  char c;			/* Reply to Continue? */
  time_t fixtime;		/* Time of position fix */
  void interrupt (*oldvect)();	/* Old interrupt vector */

  cwm = crm = buffer;
  lat = ESTLAT;
  lng = ESTLNG;
  oldvect = getvect(SIRQNO);
  setvect(SIRQNO, inface);
  outportb(IMR, (inportb(IMR) & ENIRQ));

  while (!finished)
    {
      if (findposn(getdata(), (SEAHT + GEOIDHT)))
        {
          fixtime = vardata[2].tk;
          printf("\n\n\n\n\n\n\nPosition at ");
          printf(asctime(gmtime(&fixtime)));
          printf("was %5.3f degrees East, %5.3f degrees West\n",
                 lat, lng);
        }
      printf("\n\n\n\nContinue ? : ");
      c = getchar();
      finished = ((c == 'N') || (c == 'n'));
    }

  setvect(SIRQNO, oldvect);
  exit(0);
}

/**************************************************************/

void interrupt	/* Saves doublet from PORT interface to the   */
  inface	/* input buffer. Stores LOST if buffer	      */
    (void)	/* becomes full at place where data was lost. */

		/* 16/1/89 R J Finean */

{
  if ((cwm != crm - 1) && (cwm != (crm - 1) + BUFFSIZE))
    {
      *cwm++ = inportb(PORT);
      if (cwm > buffer + BUFFSIZE)
	  cwm -= BUFFSIZE;
    }
  else
      *cwm = LOST;

  outportb(IRQCTRL, CLEAR);
}

/**************************************************************/

int getdata	/* Reads in raw data from input buffer,       */
    (void)	/* forming numeric values for data structures */
		/* and checking for errors. Returns number of */
		/* points for which data is available.	      */

		/* 17/1/89, 13/2/89 R J Finean */

{
  struct mess messdat[MAXPTS];	/* Message data workspace */
  struct fdat rawfdat[MAXPTS];	/* Fixed parameters workspace */
  struct vdat rawvdat[MAXPTS][8];
				/* Var parameters workspace */
  int data[9];			/* Data word */
  int message;			/* Message number */
  int word;			/* Word number */
  int insync;			/* Sync flag */

  printf("Waiting ...\n");
  while (!findsync(NOLIMIT))
      ;
  insync = TRUE;
  message = 0;
  while (insync && (message < MAXPTS))
    {
      printf("Found message %d\n", message);
      time(&messdat[message].rxtime);
      word = 3;
      messdat[message].dopcount = getdop();
      while ((insync = getword(data)) && (word < 156))
	{
	  word++;
	  printf("        word %d\n", word);
	  storwrd(data, word, message, rawfdat, rawvdat);
	  if (word == 128)
	      message++;
	}
      if (insync)
	  insync = findsync(58);
    }
  printf("Lost satellite. Compiling data ...\n");
  return (compile(message, rawfdat, rawvdat, messdat));
}

/**************************************************************/

void storwrd	/* Interprets data for appropriate word and  */
   (data,	/* stores a real value in rawfdat or rawvdat */
    word,	/* corresponding to current message.	     */
    message,
    rawfdat,	/* 17/1/89, 6/2/89 R J Finean */
    rawvdat)

int *data;		/* Data word */
int word;		/* Current word number */
int message;		/* Current message number */
struct fdat rawfdat[MAXPTS];
			/* Fixed data storage */
struct vdat rawvdat[MAXPTS][8];
			/* Variable data storage */

{
  int interval; 		/* Variable param interval */

  switch (word)
    {
      case 8:
      case 14:
      case 20:
      case 26:
      case 32:
      case 38:
      case 44:
      case 50:
	  interval = (word - 8) / 6;
	  rawvdat[message][interval].tk
	      = data[2] + 10 * ((*data>>2) & 0x1);
	  rawvdat[message][interval].deltaek
	      = *data & 0x2
		  ? -1E-4 * dbc((data + 2), 3) * PI / 180
		  : 1E-4 * dbc((data + 2), 3) * PI / 180;
	  rawvdat[message][interval].deltaak
	      = *data & 0x1
		  ? -10 * dbc((data + 5), 3) / 1000
		  : 10 * dbc((data + 5), 3) / 1000;
	  rawvdat[message][interval].nk = data[9];
			    /* nk not interpreted until later */
	  break;
      case 56:
	  switch (*data)
	    {
	      case 0:
		  rawfdat[message].tp
		      = dbc((data + 1), 8) * 1E-5;
		  break;
	      case 4:
		  rawfdat[message].tp
		      = (dbc((data + 1), 8) * 1E-5) + 1000;
		  break;
	      default:
		  rawfdat[message].tp = RXERR;
	    }
	  break;
      case 62:
	  rawfdat[message].n = ((sdbc(data, 8) * 1E-8) + 3)
			       * PI / 180;
	  break;
      case 68:
	  rawfdat[message].w = sdbc(data, 8) * 1E-5 * PI / 180;
	  break;
      case 74:
	  rawfdat[message].wdot = sdbc(data, 8)
				  * 1E-8 * PI / 180;
	  break;
      case 80:
	  rawfdat[message].e0 = sdbc(data, 8) * 1E-7;
	  break;
      case 86:
	  rawfdat[message].a0 = sdbc(data, 8) / 1000;
	  break;
      case 92:
	  rawfdat[message].omega = sdbc(data, 8)
				   * 1E-5 * PI / 180;
	  break;
      case 98:
	  rawfdat[message].omegadot = sdbc(data, 8)
				      * 1E-8 * PI / 180;
	  break;
      case 104:
	  rawfdat[message].ci = sdbc(data, 8) * 1E-7;
	  break;
      case 110:
	  rawfdat[message].lambdag = sdbc(data, 8)
				     * 1E-5 * PI / 180;
	  break;
      case 116:
	  rawfdat[message].id = sdbc(data, 5) * 10;
	  break;
      case 128:
	  rawfdat[message].si = sdbc(data, 8) * 1E-7;
    }
}

/**************************************************************/

int sdbc	/* Signed decimal to binary conversion of   */
   (data,	/* data words. Errors in the sign bit cause */
    length)	/* RXERR to be returned.		    */

		/* 17/1/89 R J Finean */

int *data;		/* Array of decimal integers */
int length;		/* No. of decimal digits to convert */

{
  switch (*data)
    {
      case 8:
	  return (dbc((data + 1), length));
      case 9:
	  return (0 - dbc((data + 1), length));
      default:
	  return (RXERR);
    }
}

/**************************************************************/

int dbc 	/* Decimal to binary conversion of data words */
   (data,
    length)	/* 17/1/89 R J Finean */

int *data;		/* Array of decimal integers */
int length;		/* Number of digits to convert */

{
  register int x = 0;		/* binary value */

  while (length--)
      x = (x * 10) + *data++;
  return (x);
}

/**************************************************************/

int getword	/* Reads a 39 bit word from input buffer into */
   (word)	/* word and converts it from BCDXS3 format    */
		/* into decimal. Returns OK if successful and */
		/* FAIL if PLL lock is lost or a doublet pair */
		/* cannot be decoded. For BCD errors function */
		/* returns OK, but digits in error will be    */
		/* set to RXERR.			      */

		/* 17/1/89, 14/2/89 R J Finean */

int *word;		/* Pointer to returned word */

{
  int bit;			/* Bit read */
  int digit;			/* Accumulated digit */
  int d, b;			/* Digit & bit loop indicies */

  for (d = 9; d--; )
    {
      digit = 0;
      for (b = 4; b--; )
	  if ((bit = getbit()) >= NOLOCK)
	      return (FAIL);
	  else
	      digit = (digit<<1) + bit;
      if ((digit < 13) && (digit > 2))
	  *word++ = digit - 3;
      else
	  *word++ = RXERR;
    }
  for (b = 3; b--; )
      if (getbit() >= NOLOCK)
	  return (FAIL);
  return (OK);
}

/**************************************************************/

int getbit	/* Returns a bit (2 doublets) from input. */
   (void)	/* Returns NOLOCK if PLL lock is lost, or */
		/* DOUBERR if the bit cannot be decoded.  */

		/* 17/1/89 R J Finean */

{
  int x, y;			/* 1st & 2nd doublets of bit */

  if ((x = getdoub()) >= NOLOCK)
      return (NOLOCK);
  if ((y = getdoub()) >= NOLOCK)
      return (NOLOCK);
  return (x == y ? DOUBERR : x);
}

/**************************************************************/

int findsync	/* Searches for sync word. Returns OK when    */
   (timeout)	/* found and FAIL if not found after timeout  */
		/* doublets or if PLL lock is lost.	      */
		/* If timeout set to NOLIMIT, continues until */
		/* sync word found or PLL lock is lost.	      */

		/* 23/1/89, 14/3/89 R J Finean */

int timeout;		/* Timeout period in doublets */

{
  unsigned int s0 = 0, s1 = 0,
	       s2 = 0, s3 = 0;	/* Shift registers */
  int doublet;			/* New doublet */

  while (timeout)
    {
      if ((doublet = getdoub()) >= NOLOCK)
	  return (FAIL);
      else
	{
	  s3 = (s3<<1) + ((s2>>16) & 0x0001);
	  s2 = (s2<<1) + ((s1>>16) & 0x0001);
	  s1 = (s1<<1) + ((s0>>16) & 0x0001);
	  s0 = (s0<<1) + doublet;
	  if (((s0 & 0xFFFF) == SYNC0) &&
	      ((s1 & 0xFFFF) == SYNC1) &&
	      ((s2 & 0xFFFF) == SYNC2) &&
	      ((s3 & 0x000F) == SYNC3))
		  return (OK);
	}
      if (timeout != -1)
          timeout--;
    }
  return (FAIL);
}

/**************************************************************/

unsigned long	/* Returns doppler frequency count when   */
  getdop	/* called at the start of word 3. Returns */
    (void)	/* FAIL if lock is lost or if bits 33 to  */
		/* 39 are not 0.			  */

		/* 17/1/89 R J Finean */

{
  unsigned long count = 0;	/* Doppler count */
  int bit;			/* Bit from input */
  int b;			/* Bit index number */

  for (b = 1; b < 32; b++)
      if ((bit = getdoub()) >= NOLOCK)
	  return (FAIL);
      else
	  count = (count<<1) + bit;
  for ( ; b < 39; b++)
      if (getdoub() != 0)
	  return (FAIL);
  return (count);
}

/**************************************************************/

int getdoub	/* Returns a doublet from the input buffer.   */
   (void)	/* Returns NOLOCK if PLL lock is lost or LOST */
		/* if data is missing because the buffer      */
		/* became full. 			      */

		/* 16/1/89, 13/3/89 R J Finean */

{
  time_t stime;			/* Function start time */
  register int doublet; 	/* Value to be returned */

  time(&stime);
  while (time(NULL) - stime < 2)
    {
      if (crm != cwm)
	{
	  doublet = ((*crm++)>>4) & 0x01;
	  if (crm > buffer + BUFFSIZE)
	      crm -= BUFFSIZE;
	  return (doublet);
	}
    }
  return (NOLOCK);
}

/**************************************************************/

int compile	/* Compiles data contained in rawfdat,	      */
   (messages,	/* rawvdat and messdat arrays into fixdata    */
    rawfdat,	/* and vardata. Checks for and corrects any   */
    rawvdat,	/* data errors. Returns number of points for  */
    messdat)	/* which variable parameter data is available.*/
		/* N.B. -- Verdict function treats fdat and   */
		/* vdat data structures as arrays of type     */
		/* double. Inclusion of data types other than */
		/* double in these structures will cause this */
		/* routine to fail.			      */

		/* 22/1/89, 15/2/89 R J Finean */

int messages;		/* Number of messages to scan */
struct fdat rawfdat[MAXPTS];
			/* Raw fixed data array */
struct vdat rawvdat[MAXPTS][8];
			/* Raw variable data matrix */
struct mess messdat[MAXPTS];
			/* Raw message data array */

{
  int t;			/* Var data fiducial time */
  int alts;			/* Var data alternatives */
  int x, y;			/* Var data start co-ords */
  int nki;			/* Interpreted nk */

  if (messages < 3)
      return (0);
  fixdata.tp = verdict(messages, FDSS, &rawfdat[0].tp);
  fixdata.n = verdict(messages, FDSS, &rawfdat[0].n);
  fixdata.w = verdict(messages, FDSS, &rawfdat[0].w);
  fixdata.wdot = verdict(messages, FDSS, &rawfdat[0].wdot);
  fixdata.e0 = verdict(messages, FDSS, &rawfdat[0].e0);
  fixdata.a0 = verdict(messages, FDSS, &rawfdat[0].a0);
  fixdata.omega = verdict(messages, FDSS, &rawfdat[0].omega);
  fixdata.omegadot
      = verdict(messages, FDSS, &rawfdat[0].omegadot);
  fixdata.ci = verdict(messages, FDSS, &rawfdat[0].ci);
  fixdata.lambdag
      = verdict(messages, FDSS, &rawfdat[0].lambdag);
  fixdata.id = verdict(messages, FDSS, &rawfdat[0].id);
  fixdata.si = verdict(messages, FDSS, &rawfdat[0].si);

  nki = nkdec[verdict(min(3, messages), VINT,
		      &rawvdat[2][0].nk)];
  for (t = 0; t < (messages - 1); t++)
    {
      alts = min(min(t + 4, messages), 8);
      x = max(t - 4, 0);
      y = min(t + 3, 8);
      vardata[t].tk = verdict(alts, VINT, &rawvdat[x][y].tk)
		      + (messdat[t].rxtime / 15) * 15;
      if (messdat[t].rxtime - vardata[t].tk > 7)
	  vardata[t].tk += 15;
      vardata[t].deltaek
	  = verdict(alts, VINT, &rawvdat[x][y].deltaek);
      vardata[t].deltaak
	  = verdict(alts, VINT, &rawvdat[x][y].deltaak);
      if ((int)vardata[t].tk % 4)
	{
	  nki = nkdec[verdict(alts, VINT, &rawvdat[x][y].nk)];
	  vardata[t].nk = RXERR;
	}
      else
	{
	  nki += verdict(alts, VINT, &rawvdat[x][y].nk);
	  vardata[t].nk = (nki >= 100 ? 100 - nki : nki) / 100;
	}
      if (t)
	{
	  vardata[t].dc
	      = messdat[t].dopcount - messdat[t - 1].dopcount;
	  if (vardata[t].dc < 0)
	      vardata[t].dc += 0xFFFF;
	}
    }
  for (t = 0; t < (messages - 1); t++)
      if (vardata[t].nk == RXERR)
	  if (t == messages - 1)
	      vardata[t].nk = vardata[t - 1].nk;
	  else
	      vardata[t].nk = vardata[t + 1].nk;
  return (messages);
}

/**************************************************************/

double verdict	/* Returns most popular value in a list of    */
   (messages,	/* data, starting at datap and continuing for */
    interval,	/* 'messages' items, each being separated by  */
    datap)	/* 'interval' values, all of type 'double'.   */

		/* 23/1/89, 13/2/89 R J Finean */

int messages;		/* Number of messages to scan */
int interval;		/* Interval between data items */
double *datap;		/* Pointer to 1st data item */

{
  int no;			/* Number of alternatives */
  int item;			/* Item loop index */
  register int alt;		/* Alternatives loop index */
  int found;			/* Found alternative flag */
  double lookupv[MAXPTS];	/* Alternative values table */
  int lookupn[MAXPTS];		/* 'Votes' for alternatives */

  lookupv[0] = *datap;
  lookupn[0] = 1;
  no = 1;
  for (item = 1; item < (messages - 1); item++)
    {
      for (alt = 0, found = FALSE; (alt < no) && !found; alt++)
	  if (found
	      = (*(datap + (interval * item)) == lookupv[alt]))
		  lookupn[alt]++;
      if (!found && (*(datap + (interval * item)) < RXERR))
	{
	  lookupv[no] = *(datap + interval * item);
	  lookupn[no++] = 1;
	}
    }
  for (alt = 1; alt < no; alt++)
    if (lookupn[alt] > *lookupn)
      {
	*lookupv = lookupv[alt];
	*lookupn = lookupn[alt];
      }
  return (*lookupv);
}

/**************************************************************/

int findposn	/* Uses satellite data stored in fixdata and */
   (points,	/* vardata, along with an estimate of	     */
    height)	/* position in lat and lng to determine the  */
		/* antenna position, returning latitude and  */
		/* longitude in lat and lng, respectively.   */
		/* Returns OK if sucessfull, FAIL if not.    */

		/* 14/2/89  3/3/89  R.S.Barker */

int points;		/* Number of available points */
double height;		/* Antenna height above geoid (km) */

{
  double satperiod,	/* Period of satellite (mins) */
	 t,
	 deltatp,	/* Time between time of perigee & t0 */
	 foffset,	/* Final IF */
	 slantdiff[MAXPTS],
			/* Slant range differences from
					       Doppler counts */
	 slantt[MAXPTS],/* Theoretical slant ranges from
			      coords of navigator & satellite */
	 c[MAXPTS][4],	/* The 'C' matrix */
	 b[MAXPTS][4],	/* The 'B' matrix */
	 a[4][4],	/* The 'A' matrix */
	 aa[4][4],	/* Cofactors of 'A' matrix */
	 determ,	/* Determinant of 'A' matrix */
	 deltaf,	/* Adjustment to offset frequency */
	 deltalat,	/* Adjustment to nav's latitude */
	 deltalng;	/* Adjustment to nav's longitude */
  int t0,		/* Correct time of 1st fiducial point */
      brows,		/* No of useable rows in B matrix */
      useable[MAXPTS],	/* List of such rows */
      count,i,k,p,q,	/* General bog-standard counters */
      iterations = 0;	/* Iteration counter */
  struct coords navpos, satpos[MAXPTS];
  struct partial dx, dy, dz, ds[MAXPTS];

  brows = 0;
  for (k = 1; k < points; k++)
    {
      useable[k] = (vardata[k].dc > vardata[k-1].dc);
      if (useable[k])
	  brows++;
    }
  if (brows < 3)
    {
      printf("Too few valid Doppler counts\n");
      return (FAIL);
    }

  satperiod = 2 * PI / fixdata.n;
  t0 = (int)vardata[0].tk % HOURS24;
  t = t0 - fixdata.tp;
  deltatp = (t <= -HOURS8)
            ? t + HOURS24
            : (t >= (HOURS24 - satperiod) ? t - HOURS24 : t);

  satpoints (deltatp, points, satpos);

  foffset = INTFREQ;	       /* Initial offset frequency */
  printf("Calculating fix ");

  do
    {
      printf(".");
      for (k = 1; k < points; k++)
	   slantdiff[k] = vardata[k].dc * L0
			  - 2.0 * foffset * L0;
      navpoints(height, &navpos, &dx, &dy, &dz);
      slants(points,&navpos,satpos,&dx,&dy,&dz,ds,slantt);
      for (k = 1; k < points; k++)
	{
	  c[k][0] = slantdiff[k] - slantt[k] + slantt[k-1];
	  c[k][1] = -2.0 * L0;
	  c[k][2] = ds[k-1].bylat - ds[k].bylat;
	  c[k][3] = ds[k-1].bylng - ds[k].bylng;
	}

      for (i = k = 1; k < points; k++)
	   if (useable[k])
	     {
	       for (count = 0; count < 4; count++)
		   b[i][count] = c[k][count];
	       i++;
	     }

      for (p = 1; p < 4; p++)
	   for (q = 0; q <= p; q++)
	     {
		a[p][q] = 0;
		for (i = 1; i <= brows; i++)
		    a[p][q] += b[i][p] * b[i][q];
	     }
      a[1][2] = a[2][1];
      a[1][3] = a[3][1];
      a[2][3] = a[3][2];

      aa[1][1] = a[2][2] * a[3][3] - a[3][2] * a[2][3];
      aa[2][1] = a[1][2] * a[3][3] - a[3][2] * a[1][3];
      aa[3][1] = a[1][2] * a[2][3] - a[2][2] * a[1][3];
      aa[1][2] = a[2][1] * a[3][3] - a[3][1] * a[2][3];
      aa[2][2] = a[1][1] * a[3][3] - a[3][1] * a[1][3];
      aa[3][2] = a[1][1] * a[2][3] - a[2][1] * a[1][3];
      aa[1][3] = a[2][1] * a[3][2] - a[3][1] * a[2][2];
      aa[2][3] = a[1][1] * a[3][2] - a[3][1] * a[1][2];
      aa[3][3] = a[1][1] * a[2][2] - a[2][1] * a[1][2];

      determ = a[1][1] * aa[1][1] - a[2][1] * aa[2][1]
	       + a[3][1] * aa[3][1];
      deltaf = (-a[1][0] * aa[1][1] + a[2][0] * aa[2][1]
		- a[3][0] * aa[3][1]) / determ;
      deltalat = (a[1][0] * aa[1][2] - a[2][0] * aa[2][2]
		  + a[3][0] * aa[3][2]) / determ;
      deltalng = (-a[1][0] * aa[1][3] + a[2][0] * aa[2][3]
		  - a[3][0] * aa[3][3]) / determ;

      foffset += deltaf;
      lat += deltalat;
      lng += deltalng;

      iterations++;
    }
  while ((deltalat > LATTOL || deltalng > LNGTOL ||
	 deltaf > FTOL) && (iterations < MAXITS));
  return (OK);
}

/**************************************************************/

void satpoints		/* Calculates the position of the     */
   (deltatp,		/* satellite at fiducial time wrt the */
    points,		/* navigator's x, y and z axes.       */
    satpos)
			/* 22/2/89  3/3/89  R.S.Barker */

double deltatp; 	/* Time between time of perigee and
				    t0, the 1st fiducial time */
int points;		/* Number of available points */
struct coords satpos[]; /* Satellite coords for each time */

{
  int k;
  double mm,
	 ee,
	 aa,
	 deltat,
	 u, v, w,
	 omega1,
	 x1, y1, z1,
	 beta, cosbeta, sinbeta;

  for (k = 0; k < points; k++)
    {
      deltat = deltatp + 2 * k;
      mm = fixdata.n * deltat;
      ee = mm + fixdata.e0 * sin(mm) + vardata[k].deltaek;
      aa = fixdata.a0 + vardata[k].deltaak;
      u = aa * (cos(ee) - fixdata.e0);
      v = aa * sin(ee);
      w = vardata[k].nk;
      omega1 = fixdata.w - fixdata.wdot * deltat;
      x1 = u * cos(omega1) - v * sin(omega1);
      y1 = u * sin(omega1) + v * cos(omega1);
      z1 = w;

      beta = fixdata.omega - fixdata.lambdag +
	     (fixdata.omegadot - WEARTH) * deltat;
      sinbeta = sin(beta);
      cosbeta = cos(beta);

      satpos[k].x = x1 * cosbeta - y1 * fixdata.ci * sinbeta
		    + z1 * fixdata.si * sinbeta;
      satpos[k].y = x1 * sinbeta + y1 * fixdata.ci * cosbeta
		    - z1 * fixdata.si * cosbeta;
      satpos[k].z = y1 * fixdata.si + z1 * fixdata.ci;
    }
}

/**************************************************************/

void navpoints		/* Takes navigator's lat,long,height  */
   (height,		/* to calculate x,y,z coordinates and */
    navpos,		/* their partial differentials with   */
    dx, dy, dz)		/* respect to latitude and longitude. */

			/* 22/2/89  3/3/89  R.S.Barker */

double height;		/* Navigator's height above geoid */
struct coords *navpos;	/* Navigator's x, y and z coordinates */
struct partial *dx, *dy, *dz;
			/* Partial derivatives of x, y and z
				   wrt latitude and longitude */

{
  double dist,		/* Distance centre of Earth to nav */
	 coslat, coslng,
	 sinlat, sinlng,
	 temp;

  coslat = cos(lat);
  coslng = cos(lng);
  sinlat = sin(lat);
  sinlng = sin(lng);

  dist = REARTH * sqrt(sqr(coslat) + sqr(1-FLAT) * sqr(sinlat));

  navpos->x = (sqr(REARTH) / dist + height) * coslat * coslng;
  navpos->y = (sqr(REARTH) / dist + height) * coslat * sinlng;
  navpos->z = (sqr(REARTH * (1-FLAT)) / dist + height) * sinlat;

  temp = sqr(sqr(REARTH) * (1-FLAT)) / cube(dist) + height;
  dx->bylat = -temp * sinlat * sinlng;
  dy->bylat = -temp * sinlat * sinlng;
  dz->bylat = temp * coslat;
  dx->bylng = -navpos->y;
  dy->bylng = navpos->x;
}

/**************************************************************/

void slants		/* Computes theoretical slant ranges */
   (points, navpos,	/* between navigator and each	     */
   satpos, dx, dy,	/* satellite position.		     */
   dz, ds, slantt)
   			/* 22/2/89  3/3/89  R.S.Barker */

int points;			/* Number of satellite fixes */
struct coords *navpos,		/* Navigator's position */
	      satpos[]; 	/* Satellite's positions */
struct partial *dx, *dy, *dz, ds[];
				/* Partial derivatives wrt
				       latitude and longitude */
double slantt[];		/* Theoretical slant ranges */

{
  int k;
  double xdiff, ydiff, zdiff;	/* x,y,z components of slant */

  for (k = 0; k < points; k++)
    {
	xdiff = satpos[k].x - navpos->x;
	ydiff = satpos[k].y - navpos->y;
	zdiff = satpos[k].z - navpos->z;

	slantt[k] = sqrt(sqr(xdiff) + sqr(ydiff) + sqr(zdiff));
	ds[k].bylat = -(xdiff * dx->bylat + ydiff * dy->bylat
			+ zdiff * dz->bylat) / slantt[k];
	ds[k].bylng = -(xdiff * dx->bylng + ydiff * dy->bylng)
		      / slantt[k];
    }
}
