arduino
=======

This repository contains the files and other additions for the 
'hardware/arduino' sub-directory in the arduino development environment 
that are necessary for supporting specific ATXmega CPU's.

This software is provided 'as-is' with no warranty, implied or otherwise


<p align="center"* Installing the XMegaForArduino board support *</p>

On versions of the IDE as of 1.6 you should be able to use the downloadable
hardware manager configuration file at the following location:

&nbsp;&nbsp;&nbsp;&nbsp;
<a href="https://raw.githubusercontent.com/XMegaForArduino/IDE/master/package_XMegaForArduino_index.json">
hardware manager URL for package_XMegaForArduino_index.json</a>

In some cases, you might want to do this manually, by downloading the latest
'tarball' of the distribution files, and extracting them directly into the

&nbsp;&nbsp;&nbsp;&nbsp;sketchbook/hardware

directory tree.  In some cases, this may be the preferable method.

The distribution archives are located on the IDE repository, with names similar
to the following:

&nbsp;&nbsp;&nbsp;&nbsp;XMegaForArduino-0.9.1.snapshot.txz

where '0.9.1' would be the release snapshot version.  Note that the 'tar.bz2'
files are used by the JSON file, and don't have the correct directory tree.


For earlier IDE versions (prior to 1.6), you will need to patch boards.txt
and avrdude.conf manually, and probably patch avr-gcc, avr-binutils, and
avr-libc .  The patch files are available in the 'patches' repository as
a part of this project.



NOTE:  in some cases there may be USB vendor IDs or product IDs that have
been placed into the code FOR DEMONSTRATION PURPOSES ONLY.  Under NO
circumstances are you to ship ANY products with these IDs in place.  They
are only assigned for demonstration purposes, primarily so that the code
will compile without errors or user-side edits.

If your 'FOSS' product needs a vendor ID and product ID, you might try

&nbsp;&nbsp;http://wiki.openmoko.org/wiki/USB_Product_IDs



<p align="center">* DERIVED OPEN SOURCE SOFTWARE *</p>

This software has been modified from Arduino 1.06 source that is covered by
either [L]GPLv2 or [L]GPLv3.  The licensing details are generally contained 
within the files themselves.  However, if there are no licensing
specifications, it should be treated as being the same as for the Arduino 1.06
environment (and later, as appropriate).  It is also available free of charge.

For more information, see http://arduino.cc/ and relevant source files.


<p align="center">* ADDITIONAL LICENSING FOR XMEGA-SPECIFIC CODE - MIT-like license *</p>

Portions of this code were written SPECIFICALLY for the XMEGA series processor
and are not actually 'derived code', but are included in the source as part of
a 'derived work'.  As such, within the context of the Arduino build
environment, and with respect to the implementation of the supporting
functions, the entire work should be considered a 'derived work'.

HOWEVER, if you were to make copies of ONLY the XMEGA-specific code for your
own purpose, you may license use of ONLY the XMEGA-specific code, which does
not appear in the original Arduino files from which this work was derived, to
use in your own work, provided that you include a copyright statement similar
to the following:


&nbsp;&nbsp;Portions of this work have been used by permission from the XMegaForArduino<br>
&nbsp;&nbsp;project and are Copyright (c) 2015 by S.F.T. Inc. - all rights reserved


So if you simply included portions of the 'clock setup' code for the XMEGA
processor within your own work, you could license that under an MIT-like
as stated above, in lieu of an [L]GPLv2 or [L]GPLv3 license (as appropriate).
Keep in mind that inclusion of actual GPL-derived code would not be allowed,
so at the most you could copy the XMEGA-specific code, but not the entire
function, nor implement something that could be considered 'derived from' the
GPL version of the code.  This distinction has been provided specifically so
that you can use this project as 'sample code' for your own work, for doing
some of the more difficult tasks (like setting up the clock), without being
locked into using GPL for your own code afterwards.

If you believe that your resulting work MIGHT STILL be a derived work, you
should consider using the appropriate [L]GPL license.


