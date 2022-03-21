/* Spartan3 JTAG programmer

Copyright (C) 2004 Andrew Rogers
              2005-2011 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de

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

Changes:
Dmitry Teytelman [dimtey@gmail.com] 14 Jun 2006 [applied 13 Aug 2006]:
    Code cleanup for clean -Wall compile.
    Added support for FT2232 driver.
    Verbose support added.
    Installable device database location.
*/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <list>
#include <memory>
#include <errno.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <tcl/tcl.h>

#include "io_exception.h"
#include "ioparport.h"
#include "iofx2.h"
#include "ioftdi.h"
#include "ioxpc.h"
#include "sysfs.h"
#include "bitfile.h"
#include "jtag.h"
#include "devicedb.h"
#include "cabledb.h"
#include "progalgxcf.h"
#include "progalgxcfp.h"
#include "javr.h"
#include "progalgxc3s.h"
#include "jedecfile.h"
#include "mapfile_xc2c.h"
#include "progalgxc95x.h"
#include "progalgxc2c.h"
#include "progalgavr.h"
#include "progalgspiflash.h"
#include "progalgnvm.h"
#include "utilities.h"

using namespace std;

#define MAXPOSITIONS    8

#define IDCODE_TO_FAMILY(id)        ((id>>21) & 0x7f)
#define IDCODE_TO_MANUFACTURER(id)  ((id>>1) & 0x3ff)

#define MANUFACTURER_ATMEL          0x01f
#define MANUFACTURER_XILINX         0x049

    int do_exit = 0;
void ctrl_c(int sig)
{
  do_exit = 1;
}


unsigned long get_id(std::auto_ptr<Jtag> &jtag, DeviceDB &db, int chainpos)
{
  bool verbose = jtag->getVerbose();
  int num = jtag->getChain();
  if (jtag->selectDevice(chainpos)<0)
    {
      fprintf(stderr, "Invalid chain position %d, must be >= 0 and < %d\n",
              chainpos, num);
      return 0;
    }
  unsigned long id = jtag->getDeviceID(chainpos);
  if (verbose)
    {
      fprintf(stderr, "JTAG chainpos: %d Device IDCODE = 0x%08lx\tDesc: %s\n",
              chainpos, id, db.idToDescription(id));
      fflush(stderr);
    }
  return id;
}
  
  
void usage(bool all_options)
{
  fprintf(stderr, "usage:\txc3sprog -c cable [options] <file0spec> <file1spec> ...\n");
  fprintf(stderr, "\tList of known cables is given with -c follow by no or invalid cablename\n");
  fprintf(stderr, "\tfilespec is filename:action:offset:style:length\n");
  fprintf(stderr, "\taction on of 'w|W|v|r|R'\n");
  fprintf(stderr, "\tw: erase whole area, write and verify\n");
  fprintf(stderr, "\tW: Write with auto-sector erase and verify\n");
  fprintf(stderr, "\tv: Verify device against filename\n");
  fprintf(stderr, "\tr: Read from device,write to file, don't overwrite existing file\n");
  fprintf(stderr, "\tR: Read from device and write to file, overwrite existing file\n");
  fprintf(stderr, "\tDefault action is 'w'\n\n");
  fprintf(stderr, "\tDefault offset is 0\n\n");
  fprintf(stderr, "\tstyle: One of BIT|BIN|BPI|MCS|IHEX|HEX\n");
  fprintf(stderr, "\tBIT: Xilinx .bit format\n");
  fprintf(stderr, "\tBIN: Binary format\n");
  fprintf(stderr, "\tBPI: Binary format not bit reversed\n");
  fprintf(stderr, "\tMCS: Intel Hex File, LSB first\n");
  fprintf(stderr, "\tIHEX: INTEL Hex format, MSB first (Use for Xilinx .mcs files!)\n");
  fprintf(stderr, "\tHEX:  Hex dump format\n");
  fprintf(stderr, "\tDefault for FPGA|SPI|XCF is BIT\n");
  fprintf(stderr, "\tDefault for CPLD is JED\n");
  fprintf(stderr, "\tDefault for XMEGA is IHEX\n");
  fprintf(stderr, "\tDefault length is whole device\n");

  if (!all_options) exit(255);

  fprintf(stderr, "\nPossible options:\n");
#define OPT(arg, desc)	\
  fprintf(stderr, "   %-8s  %s\n", (arg), (desc))
  OPT("-p val[,val...]", "Use device at JTAG Chain position <val>.");
  OPT("",   "Default (0) is device connected to JTAG Adapter TDO.");
  OPT("-e", "Erase whole device.");
  OPT("-h", "Print this help.");
  OPT("-I[file]", "Work on connected SPI Flash (ISF Mode),");
  OPT(""  , "after loading 'bscan_spi' bitfile if given.");
  OPT("-j", "Detect JTAG chain, nothing else (default action).");
  OPT("-l", "Program lockbits if defined in fusefile.");
  OPT("-m <dir>", "Directory with XC2C mapfiles.");
  OPT("-R", "Try to reconfigure device(No other action!).");
  OPT("-T val", "Test chain 'val' times (0 = forever) or 10000 times"
      " default.");
  OPT("-J val", "Run at max with given JTAG Frequency, 0(default) means max. Rate of device");
  OPT("", "Only used for FTDI cables for now");
  OPT("-D", "Dump internal devlist and cablelist to files");
  OPT(""      , "In ISF Mode, test the SPI connection.");
  OPT("-X opts", "Set options for XCFxxP programming");
  OPT("-v", "Verbose output.");

  fprintf(stderr, "\nProgrammer specific options:\n");
  /* Parallel cable */
  OPT("-d", "(pp only     ) Parallel port device.");
  /* USB devices */
  OPT("-s num" , "(usb devices only) Serial number string.");
  OPT("-L     ", "(ftdi only       ) Don't use LibUSB.");

  fprintf(stderr, "\nDevice specific options:\n");
  OPT("-E file", "(AVR only) EEPROM file.");
  OPT("-F file", "(AVR only) File with fuse bits.");
#undef OPT

  exit(255);
}

#if 0
void dump_lists(CableDB *cabledb, DeviceDB *db)
{
    int fd;
    FILE *fp_out;
    char outname[17] = "cablelist.txt";

   fd = open(outname, O_WRONLY|O_CREAT|O_EXCL, 0644);
    if (fd <0 )
    {
        sprintf(outname,"cablelist.txt.1");
        fd = open(outname, O_WRONLY|O_CREAT, 0644);
    }
    if (fd <0 )
    {
        fprintf(stderr, "Error creating file\n");
    }
    else
    {
        fprintf(stderr, "Dumping internal cablelist to %s\n", outname);
        fp_out = fdopen(fd, "w");
        if (fp_out)
        {
            fprintf(fp_out, "%-20s%-8s%-10sOptString\n", "#Alias", "Type", "Frequency");
            cabledb->dumpCables(fp_out);
            fclose(fp_out);
        }
    }

    sprintf(outname,"devlist.txt");
   fd = open(outname, O_WRONLY|O_CREAT|O_EXCL, 0644);
    if (fd <0 )
    {
        sprintf(outname,"devlist.txt.1");
        fd = open(outname, O_WRONLY|O_CREAT, 0644);
    }
    if (fd <0 )
    {
        fprintf(stderr, "Error creating file\n");
    }
    else
    {
        fprintf(stderr, "Dumping internal devicelist to %s\n", outname);
        fp_out = fdopen(fd, "w");
        if (fp_out)
        {
            fprintf(fp_out, "# IDCODE IR_len ID_Cmd Text\n");
            db->dumpDevices(fp_out);
            fclose(fp_out);
        }
    }
    exit(0);
}

int main(int argc, char **args)
{
  bool        verbose   = false;
  bool        dump      = false;
  bool        verify    = false;
  bool        lock      = false;
  bool     detectchain  = false;
  bool     chaintest    = false;
  bool     spiflash     = false;
  bool     reconfigure  = false;
  bool     erase        = false;
  bool     use_ftd2xx   = false;
  unsigned int jtag_freq= 0;
  unsigned long id;
  struct cable_t cable;
  char const *dev       = 0;
  char const *eepromfile= 0;
  char const *fusefile  = 0;
  char const *mapdir    = 0;
  FILE_STYLE in_style  = STYLE_BIT;
  FILE_STYLE out_style = STYLE_BIT;
  int      chainpos     = 0;
  int      nchainpos    = 1;
  int      chainpositions[MAXPOSITIONS] = {0};
  vector<string> xcfopts;
  int test_count = 0;
  char const *serial  = 0;
  char *bscanfile = 0;
  char *cablename = 0;
  char osname[OSNAME_LEN];
  DeviceDB db(NULL);
  CableDB cabledb(NULL);
  std::auto_ptr<IOBase>  io;
  int res;

  get_os_name(osname, sizeof(osname));
  // Produce release info from SVN tags
  fprintf(stderr, "XC3SPROG (c) 2004-2011 xc3sprog project $Rev: 774 $ OS: %s\n"
	  "Free software: If you contribute nothing, expect nothing!\n"
	  "Feedback on success/failure/enhancement requests:\n"
          "\thttp://sourceforge.net/mail/?group_id=170565 \n"
	  "Check Sourceforge for updates:\n"
          "\thttp://sourceforge.net/projects/xc3sprog/develop\n\n",
	  osname);

  // Start from parsing command line arguments
  while(true) {
      int c = getopt(argc, args, "?hCLc:d:DeE:F:i:I::jJ:Lm:o:p:Rs:S:T::vX:");
    switch(c) 
    {
    case -1:
      goto args_done;

    case 'v':
      verbose = true;
      break;

    case 'j':
      detectchain = true;
      break;

    case 'L':
      use_ftd2xx = true;
      break;

    case 'J':
      jtag_freq = atoi(optarg);
      break;

    case 'c':
      cablename =  optarg;
      break;

    case 'm':
      mapdir = optarg;
      break;
      
     case 'd':
      dev = optarg;
      break;

    case 'p':
      {
        char *p = optarg, *q;
        for (nchainpos = 0; nchainpos <= MAXPOSITIONS; )
          {
            chainpositions[nchainpos] = strtoul(p, &q, 10);
            if (p == q)
              break;
            p = q;
            nchainpos++;
            if (*p == ',')
              p++;
            else
              break;
          }
        if (*p != '\0')
          {
            fprintf(stderr, "Invalid position specification \"%s\"\n", optarg);
            usage(false);
          }
      }
      chainpos = chainpositions[0];
      break;

    case '?':
    case 'h':
    default:
        if (optopt == 'c')
        {
            fprintf(stdout, "Known Cables\n");
            cabledb.dumpCables(stderr);
            exit(1);
        }
        fprintf(stderr, "Unknown option -%c\n", c);
        usage(true);
    }
  }
 args_done:
  argc -= optind;
  args += optind;
  if (dump)
      dump_lists(&cabledb, &db);

  if((argc < 0) || (cablename == 0))  usage(true);
  if(argc < 1 && !reconfigure && !erase) detectchain = true;
  if (verbose)
  {
    fprintf(stderr, "Using %s\n", db.getFile().c_str());
    fprintf(stderr, "Using %s\n", cabledb.getFile().c_str());
  }
  res = cabledb.getCable(cablename, &cable);
  if(res)
  {
      fprintf(stderr,"Can't find description for a cable named %s\n",
             cablename);
      fprintf(stdout, "Known Cables\n");
      cabledb.dumpCables(stderr);
      exit(1);
  }
  
  res = getIO( &io, &cable, dev, serial, verbose, use_ftd2xx, jtag_freq);
  if (res) /* some error happend*/
    {
      if (res == 1) exit(1);
      else usage(false);
    }
  
  Jtag jtag = Jtag(io.get());
  jtag.setVerbose(verbose);

  if (init_chain(jtag, db))
    id = get_id (jtag, db, chainpos);
  else
    id = 0;

  detect_chain(&jtag, &db);

  if (id == 0)
    return 2;

  unsigned int family = IDCODE_TO_FAMILY(id);
  unsigned int manufacturer = IDCODE_TO_MANUFACTURER(id);

  unsigned char ir_in[4];
  unsigned char dr_in[4];
  unsigned char out[4];
  ir_in[3]=ir_in[2]=ir_in[1]=0x00;
  ir_in[0]=0x02;
  dr_in[3]=dr_in[2]=dr_in[1]=0x00;
  dr_in[0]=0xff;
  
  jtag.shiftIR(&ir_in[0],&out[0]);
  jtag.shiftDR(&dr_in[0],&out[0],8);
  printf("Out: %x\n",out[0]);
  dr_in[0]=out[0]/2;
  jtag.shiftDR(&dr_in[0],&out[0],8);
  printf("Out: %x\n",out[0]);

  ir_in[0]=0xff; // Bypass
  jtag.shiftIR(&ir_in[0],&out[0]);

  return 0;
}
#endif 


class TclJTAG
{
	public:
	TclJTAG(Tcl_Interp *interp)
		: connected(false), devicedb(NULL), cabledb(NULL), jtag(NULL), jtag_freq(0), dev(0), use_ftd2xx(false), serial(0), verbose(false)
	{
		if (Tcl_InitStubs(interp, "8.1", 0) == NULL) {
		    throw -1;
		}

		if ( Tcl_PkgProvide(interp, "TclJTAG", "1.0") != TCL_OK ) {
		    throw -1;
		}
		Tcl_CreateObjCommand(interp, "jtag::get_cables", getcables, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::open_cable", opencable, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::close_cable", closecable, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::select_device", selectdevice, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::get_device_id", deviceid, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::get_device_description", devicedescription, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::detect_chain", detectchain, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::shift_ir", shift_ir, this, NULL);
		Tcl_CreateObjCommand(interp, "jtag::shift_dr", shift_dr, this, NULL);
	}
	~TclJTAG()
	{
	
	}
	protected:
	// Static member functions, called via fptr
	static TclJTAG *GetJTAG(ClientData cdata)
	{
		TclJTAG *result=(TclJTAG *)cdata;
		if(cdata)
		{
			if(!result->connected)
				result=0;
		}
		if(!result)
			fprintf(stderr,"Cable not connected\n");
		return(result);
	}
	
	static int getcables(ClientData cdata,Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		TclJTAG *j=(TclJTAG *)cdata;
		Tcl_Obj *list;
		int idx=0;
		const char *alias;
		if(!j)
			return TCL_ERROR;
		list=Tcl_NewListObj(j->cabledb.getCableCount(),0);
		while(alias=j->cabledb.getCableAlias(idx++))
		{
			Tcl_Obj *str=Tcl_NewStringObj(alias,strlen(alias));
			Tcl_ListObjAppendElement(interp,list,str);
			fprintf(stderr,"Cable: %s\n",alias);			
		}
		Tcl_SetObjResult(interp,list);
		return(TCL_OK);
	}

	static int closecable(ClientData cdata,Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int res;
		TclJTAG *j=(TclJTAG *)cdata;
		if(!j)
			return TCL_ERROR;
		if(!j->connected)
		{
			fprintf(stderr,"No cable connected\n");
			return TCL_ERROR;
		}
		j->jtag.reset();
		j->io.reset();
		j->connected=false;
		return TCL_OK;
	}
	 
	static int opencable(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int res;
		TclJTAG *j=(TclJTAG *)cdata;
		if(!j)
			return TCL_ERROR;

		j->jtag.reset();
		j->io.reset();
		j->connected=false;

		switch(objc)
		{
			case 1:
				printf("FIXME - fetch list of cables...\n");
				return TCL_ERROR;
				break;
			case 2:
				res=j->cabledb.getCable(Tcl_GetString(objv[1]), &j->cable);
				if(res)
				{
					// FIXME dump a list of cables here
					fprintf(stderr,"Failed to open cable %s\n");
					return(TCL_ERROR);
				}
//				printf("Getting IO...\n");  
				res = getIO( &j->io, &j->cable, j->dev, j->serial, j->verbose, j->use_ftd2xx, j->jtag_freq);
				if (res) /* some error happend*/
				{
					fprintf(stderr,"Failed to open cable %s\n");
					return(TCL_ERROR);
				}
//				printf("Got IO, creating JTAG object...\n");  

				j->jtag.reset(new Jtag(j->io.get()));
				j->jtag->setVerbose(j->verbose);

				detect_chain(j->jtag.get(), &j->devicedb);
				if (!j->jtag->getChain())
				{
					fprintf(stderr,"Couldn't initialise chain\n");
					return(TCL_ERROR);
				}
				j->connected=true;
				Tcl_SetResult(interp,"1",TCL_STATIC);
				return(TCL_OK);
				break;
		}

		return(TCL_ERROR);
	}
	
	static int shift_ir(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int res=TCL_OK;
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;
			
		if(objc==2)
		{
			int irval;
			if((res=Tcl_GetIntFromObj(interp,objv[1],&irval))==TCL_OK)
			{
				j->irbuf[0]=irval&255;
				j->irbuf[1]=(irval>>8)&255;
				j->irbuf[2]=(irval>>16)&255;
				j->irbuf[3]=(irval>>24)&255;
				j->jtag->shiftIR(j->irbuf,j->outbuf);
			}
		}
		return(res);
	}

	static int shift_dr(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int res=TCL_OK;
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;
			
		if(objc==3)
		{
			int drval;
			int length;
			if((res=Tcl_GetIntFromObj(interp,objv[1],&drval))==TCL_OK)
			{
				if((res=Tcl_GetIntFromObj(interp,objv[2],&length))==TCL_OK)
				{
					int i;
					j->drbuf[0]=drval&255;
					j->drbuf[1]=(drval>>8)&255;
					j->drbuf[2]=(drval>>16)&255;
					j->drbuf[3]=(drval>>24)&255;
					j->jtag->shiftDR(j->drbuf,j->outbuf,length);
					drval=0;
					for(i=(length-1)/8;i>=0;--i)
					{
						if(i<4)
						{
							drval|=j->outbuf[i]<<(i*8);
						}
					}
					Tcl_SetObjResult(interp,Tcl_NewIntObj(drval));
				}
			}
		}
		else
		{
			fprintf(stderr,"Usage: shift_dr value length\n");
			return TCL_ERROR;
		}
		return(res);
	}

	static int detectchain(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;
		if(!j->connected)
		{
			fprintf(stderr,"No cable connected\n");
			return TCL_ERROR;
		}
		detect_chain(j->jtag.get(), &j->devicedb);
		if (j->jtag->getChain(1))
		{
			fprintf(stderr,"Couldn't initialise chain\n");
			return(TCL_ERROR);
		}
		return(TCL_OK);
	}

	static int selectdevice(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int chainpos=0;
		int res=0;
		int num;
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;

		if(objc>=2)
			res=Tcl_GetIntFromObj(interp,objv[1],&chainpos);

		if(res!=TCL_OK)
			return(res);			

		if(!j->connected)
		{
			fprintf(stderr,"No cable connected\n");
			return TCL_ERROR;
		}
		num=j->jtag->getChain();	
		if(chainpos<0 || chainpos>=num)
		{
			fprintf(stderr,"%d devices in chain - chainpos must be between %d and %d\n",num,0,num-1);
			return TCL_ERROR;		
		}	
		if(j->jtag->selectDevice(chainpos)<0)
		{
			fprintf(stderr,"SelectDevice failed\n");
			return TCL_ERROR;
		}
		return TCL_OK;
    }

	static int deviceid(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int chainpos=0;
		int res=0;
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;

		if(objc>=2)
			res=Tcl_GetIntFromObj(interp,objv[1],&chainpos);

		if(res!=TCL_OK)
			return(res);

		res = get_id (j->jtag, j->devicedb, chainpos);
		Tcl_SetObjResult(interp,Tcl_NewIntObj(res));
		return(TCL_OK);
	}

	static int devicedescription(ClientData cdata, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[])
	{
		int chainpos=0;
		int res=0;
		TclJTAG *j=GetJTAG(cdata);
		if(!j)
			return TCL_ERROR;

		if(objc>=2)
			res=Tcl_GetIntFromObj(interp,objv[1],&chainpos);

		if(res!=TCL_OK)
			return(res);

		res = get_id (j->jtag, j->devicedb, chainpos);
		const char *result=j->devicedb.idToDescription(res);

		Tcl_SetObjResult(interp,Tcl_NewStringObj(result,-1));
		return(TCL_OK);
	}
	protected:
	bool connected;
	DeviceDB devicedb;
	CableDB cabledb;
	struct cable_t cable;
	std::auto_ptr<Jtag> jtag;
	std::auto_ptr<IOBase> io;
	unsigned int jtag_freq;
	char const *dev;
	bool use_ftd2xx;
	char const *serial;
	bool verbose;
	unsigned char irbuf[4];
	unsigned char drbuf[4];
	unsigned char outbuf[4];
};


extern "C" {
    int Tcljtag_Init(Tcl_Interp *interp);
}

int Tcljtag_Init(Tcl_Interp *interp)
{
	try{
		TclJTAG *h=new TclJTAG(interp);
		return TCL_OK;
	}
	catch(...)
	{
		return TCL_ERROR;
	}
}


