#!/bin/sh


ID=`echo $1 | awk '{ if(substr($0,1,7)=="atxmega") print "x" substr($0,8,99); else print $0; }'`


cat <<THING >/tmp/get_sig.txt
BEGIN {
  F=0;
}
{
  A=\$0;
  while(length(A) > 0 && substr(A,1,1)<=" ")
  {
    A=substr(A,2,length(A)-1);
  }

  if(F==0)
  {
    if(substr(A,1,4)=="part")
    {
      F=1;
    }
  }
  else if(F==1)
  {
    if(substr(A,1,2)=="id" && (substr(A,3,1)=="=" || substr(A,3,1)<=" "))
    {
      A=substr(A,3,length(A)-2);

      while(substr(A,1,1) <= " " && length(A) > 0)
      {
        A=substr(A,2,length(A)-1);
      }

      if(substr(A,1,1)=="=")
      {
        A=substr(A,2,length(A)-1);

        while(substr(A,1,1) <= " " && length(A) > 0)
        {
          A=substr(A,2,length(A)-1);
        }

        if(substr(A,length(A),1) == ";")
        {
          A=substr(A,1,length(A)-1);
          while(length(A) > 0 && substr(A,length(A),1) <= " ")
          {
            A=substr(A,1,length(A)-1);
          }
        }

        if(substr(A,1,1)=="\"" && substr(A,length(A),1)=="\"")
        {
          A=substr(A,2,length(A)-2);
        }

        if(A=="$ID")
        {
          F=2;
        }
      }
    }
  }
  else if(F==2)
  {
    if(substr(A,1,4)=="part")
    {
      F=1;
    }
    else
    {
      if(substr(A,1,9) == "signature")
      {
        A = substr(A,10,length(A)-9);

        while(substr(A,1,1) <= " " && length(A) > 0)
        {
          A=substr(A,2,length(A)-1);
        }

        if(substr(A,1,1)=="=")
        {
          A=substr(A,2,length(A)-1);

          while(substr(A,1,1) <= " " && length(A) > 0)
          {
            A=substr(A,2,length(A)-1);
          }

          if(substr(A,length(A),1) == ";")
          {
            A=substr(A,1,length(A)-1);
            while(length(A) > 0 && substr(A,length(A),1) <= " ")
            {
              A=substr(A,1,length(A)-1);
            }
          }

          print A;
        }

        F=0;
      }
    }
  }

}
THING

# simply output the text

awk -f /tmp/get_sig.txt < avrdude.conf


