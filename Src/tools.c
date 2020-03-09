/**
 *	Modified 06.11.2019
 *
 *	Functions that do amazing things :)
 *	Author: Adrian Wojak
 *
 */

#include "tools.h"
//#include "main.h"

/**
 * Function convert integer number to text
 * int number - number to convert
 * char *text - pointer for array where text will be save,
 *  			should have enough space to save whole number max 12 bytes with sign and \0.
 * return - number of digit with sign but without \0
 */
int IntToChar(int number, char *text)
{
	int size = 0, dig = 0, tmp = 0;

	if(number == 0)
	{
		*text++ = '0';
		*text = '\0';
		return 1;
	}

	if(number < 0)
	{
		//add minus sign
		*text++ = '-';
		size++;
		//change sign, need for algorithm
		number *= -1;
	}

    //looking for digit count
    tmp = number;
    while(tmp>0)
    {
        tmp/=10;
        dig++;
    }
	*(text+dig) = '\0';
    dig--;
    //Change number to char
	while(number>0)
	{
		*(text+dig--)=0x30+(number%10);
		number/=10;
		size++;
	}

	return size;
}

/**
 * Need check and update
 */
unsigned char BCDToNumber(unsigned char n) {
	return ((((unsigned short)(n&0xF0)*10)>>4)+(n&0x0F));
}

/*! \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
unsigned long Sqrt(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min(unsigned int x, unsigned int y)
{
  if(x < y){
    return x;
  }
  else{
    return y;
  }
}

