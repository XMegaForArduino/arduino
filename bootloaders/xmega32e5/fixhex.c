//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                   __  _        _                                         //
//                  / _|(_)__  __| |__    ___ __  __    ___                 //
//                 | |_ | |\ \/ /| '_ \  / _ \\ \/ /   / __|                //
//                 |  _|| | >  < | | | ||  __/ >  <  _| (__                 //
//                 |_|  |_|/_/\_\|_| |_| \___|/_/\_\(_)\___|                //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//          Copyright (c) 2014 by S.F.T. Inc. - All rights reserved         //
//  Use, copying, and distribution of this software are licensed according  //
//  to the GPLv2, LGPLv2, or BSD license, as appropriate, your choice.      //
//      NO WARRANTY EITHER EXPLICIT OR IMPLIED.  USE AT YOUR OWN RISK       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

// this program will modify the hex file to change the start address from 8000H to 0000H
// so that I can flash it into the bootloader at offset 0.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>


unsigned char hexdigit(const char *p1)
{
  if(*p1 >= '0' && *p1 <= '9')
  {
    return *p1 - '0';
  }
  else if(*p1 >= 'a' && *p1 <= 'f')
  {
    return *p1 - 'a' + 10;
  }
  else if(*p1 >= 'A' && *p1 <= 'F')
  {
    return *p1 - 'A' + 10;
  }

  return 0;
}

unsigned short hexval(const char *p1)
{
  return hexdigit(p1) * 16 + hexdigit(p1 + 1);
}

int main(int argc, char *argv[])
{
FILE *pIn, *pOut;
char tbuf[256];
int i1, i2, cb1;
char *p1;
static const char szHex[]="0123456789ABCDEF";


  if(argc <= 2)
  {
    printf("usage:  fixhex infile outfile\n");
    return 1;
  }

  pIn = fopen(argv[1], "r");
  if(!pIn)
  {
    fprintf(stderr, "unable to read from %s, error %d\n", argv[1], errno);

    return -1;
  }

  pOut = fopen(argv[2], "w");
  if(!pOut)
  {
    fclose(pIn);

    fprintf(stderr, "unable to write to %s, error %d\n", argv[2], errno);
    return -2;
  }

  while(!feof(pIn))
  {
    if(!fgets(tbuf, sizeof(tbuf) - 1, pIn))
    {
      break;
    }

    // line starts with :00 or :nn for byte count

    if(tbuf[0] != ':' || strlen(tbuf) < 9 || // not a valid intel hex record
       (tbuf[1] == '0' && tbuf[2] == '0') || // length is zero
       tbuf[7] != '0' || tbuf[8] != '0')     // not a 'data' record
    {
//      printf("here I am: %c %d %c %c %c %c\n", tbuf[0], strlen(tbuf), tbuf[1], tbuf[2], tbuf[7], tbuf[8]);
      fputs(tbuf, pOut);
      continue;
    }

    // re-write the string.  1st 2 chars is byte count

    if(tbuf[3] == '8')
    {
      tbuf[3] = '0';

      // re-do checksum since I modified the data
 
      p1 = &(tbuf[1]);
      i2 = hexval(p1); // note this is also the count *and* the new running checksum
      p1 += 2;

      // next 3 '2-bytes' are the address + type [00], then 'n' bytes of data, then checksum byte + CRLF
      for(i1=0, cb1=i2;  i1 < i2 + 3; i1++, p1 += 2)
      {
        cb1 += hexval(p1); // determine the checksum
      }

      // now write the 2-digit hex value into the correct characters pointed by p1     
      // which is a 2's compliment of the sum of the length, address, type, and data
      // see https://en.wikipedia.org/wiki/.hex

      cb1 = -cb1; // 2's compliment
      *(p1++) = szHex[(cb1 & 0xf0) >> 4]; // write upper nybble to text data
      *p1 = szHex[cb1 & 0xf];             // write lower nybble to text data

//      printf("TEMP:  %02x %02x %s\n", (cb1 & 0xff), ((-cb1) & 0xff), p1 - 1);
    }

    fputs(tbuf, pOut);
  }  

  fclose(pIn);
  fclose(pOut);

  return 0;
}


