/****************************************************************************
*
*                   VBE 2.0 Linear Framebuffer Profiler
*                    By Kendall Bennett and Brian Hook
*
* Filename:     LFBPROF.C
* Language:     ANSI C
* Environment:  Watcom C/C++ 10.0a with DOS4GW
*
* Description:  Simple program to profile the speed of screen clearing
*               and full screen BitBlt operations using a VESA VBE 2.0
*               linear framebuffer from 32 bit protected mode.
*
*               For simplicity, this program only supports 256 color
*               SuperVGA video modes that support a linear framebuffer.
*
*
* 2002/02/18: Jeroen Janssen <japj at xs4all dot nl>
*               - fixed unsigned short for mode list (-1 != 0xffff otherwise)
*               - fixed LfbMapRealPointer macro mask problem (some modes were skipped)
*
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>
#include <dos.h>
/* SuperVGA information block */

typedef struct {
    char    VESASignature[4];       /* 'VESA' 4 byte signature          */
    short   VESAVersion;            /* VBE version number               */
    long    OemStringPtr;           /* Pointer to OEM string            */
    long    Capabilities;           /* Capabilities of video card       */
    long    VideoModePtr;           /* Pointer to supported modes       */
    short   TotalMemory;            /* Number of 64kb memory blocks     */

    /* VBE 2.0 extensions */

    short   OemSoftwareRev;         /* OEM Software revision number     */
    long    OemVendorNamePtr;       /* Pointer to Vendor Name string    */
    long    OemProductNamePtr;      /* Pointer to Product Name string   */
    long    OemProductRevPtr;       /* Pointer to Product Revision str  */
    char    reserved[222];          /* Pad to 256 byte block size       */
    char    OemDATA[256];           /* Scratch pad for OEM data         */
    } VBE_vgaInfo;

/* SuperVGA mode information block */

typedef struct {
    short   ModeAttributes;         /* Mode attributes                  */
    char    WinAAttributes;         /* Window A attributes              */
    char    WinBAttributes;         /* Window B attributes              */
    short   WinGranularity;         /* Window granularity in k          */
    short   WinSize;                /* Window size in k                 */
    short   WinASegment;            /* Window A segment                 */
    short   WinBSegment;            /* Window B segment                 */
    long    WinFuncPtr;             /* Pointer to window function       */
    short   BytesPerScanLine;       /* Bytes per scanline               */
    short   XResolution;            /* Horizontal resolution            */
    short   YResolution;            /* Vertical resolution              */
    char    XCharSize;              /* Character cell width             */
    char    YCharSize;              /* Character cell height            */
    char    NumberOfPlanes;         /* Number of memory planes          */
    char    BitsPerPixel;           /* Bits per pixel                   */
    char    NumberOfBanks;          /* Number of CGA style banks        */
    char    MemoryModel;            /* Memory model type                */
    char    BankSize;               /* Size of CGA style banks          */
    char    NumberOfImagePages;     /* Number of images pages           */
    char    res1;                   /* Reserved                         */
    char    RedMaskSize;            /* Size of direct color red mask    */
    char    RedFieldPosition;       /* Bit posn of lsb of red mask      */
    char    GreenMaskSize;          /* Size of direct color green mask  */
    char    GreenFieldPosition;     /* Bit posn of lsb of green mask    */
    char    BlueMaskSize;           /* Size of direct color blue mask   */
    char    BlueFieldPosition;      /* Bit posn of lsb of blue mask     */
    char    RsvdMaskSize;           /* Size of direct color res mask    */
    char    RsvdFieldPosition;      /* Bit posn of lsb of res mask      */
    char    DirectColorModeInfo;    /* Direct color mode attributes     */

    /* VBE 2.0 extensions */

    long    PhysBasePtr;            /* Physical address for linear buf  */
    long    OffScreenMemOffset;     /* Pointer to start of offscreen mem*/
    short   OffScreenMemSize;       /* Amount of offscreen mem in 1K's  */
    char    res2[206];              /* Pad to 256 byte block size       */
    } VBE_modeInfo;

#define vbeMemPK        4           /* Packed Pixel memory model        */
#define vbeUseLFB       0x4000      /* Enable linear framebuffer mode   */

/* Flags for the mode attributes returned by VBE_getModeInfo. If
 * vbeMdNonBanked is set to 1 and vbeMdLinear is also set to 1, then only
 * the linear framebuffer mode is available.
 */

#define vbeMdAvailable  0x0001      /* Video mode is available          */
#define vbeMdColorMode  0x0008      /* Mode is a color video mode       */
#define vbeMdGraphMode  0x0010      /* Mode is a graphics mode          */
#define vbeMdNonBanked  0x0040      /* Banked mode is not supported     */
#define vbeMdLinear     0x0080      /* Linear mode supported            */

/* Structures for issuing real mode interrupts with DPMI */

struct _RMWORDREGS {
    unsigned short ax, bx, cx, dx, si, di, cflag;
    };

struct _RMBYTEREGS {
    unsigned char   al, ah, bl, bh, cl, ch, dl, dh;
    };

typedef union {
    struct  _RMWORDREGS x;
    struct  _RMBYTEREGS h;
    } RMREGS;

typedef struct {
    unsigned short  es;
    unsigned short  cs;
    unsigned short  ss;
    unsigned short  ds;
    } RMSREGS;

/* Inline assembler block fill/move routines */

void LfbMemset(void *p,int c,int n) 
{
#pragma aux LfbMemset =             \
	"shr    ecx,2"                  \
	"xor    eax,eax"                \
	"mov    al,bl"                  \
	"shl    ebx,8"                  \
	"or     ax,bx"                  \
	"mov    ebx,eax"                \
	"shl    ebx,16"                 \
	"or     eax,ebx"                \
	"rep    stosd"                  \
	parm [edi] [ebx] [ecx];
}
void LfbMemcpy(void *dst,void *src,int n)
{
#pragma aux LfbMemcpy =             \
	"shr    ecx,2"                  \
	"rep    movsd"                  \
	parm [edi] [esi] [ecx];
}
/* Map a real mode pointer into address space */

#define LfbMapRealPointer(p)    (void*)(((unsigned)((p)  & 0xFFFF0000) >> 12) + ((p) & 0xFFFF))

/* Get the current timer tick count */

#define LfbGetTicks()       *((long*)0x46C)
/*---------------------------- Global Variables ---------------------------*/

int     VESABuf_len = 1024;         /* Length of VESABuf                */
int     VESABuf_sel = 0;            /* Selector for VESABuf             */
int     VESABuf_rseg;               /* Real mode segment of VESABuf     */
unsigned short   modeList[50];      /* List of available VBE modes      */
float   clearsPerSec;               /* Number of clears per second      */
float   clearsMbPerSec;             /* Memory transfer for clears       */
float   bitBltsPerSec;              /* Number of BitBlt's per second    */
float   bitBltsMbPerSec;            /* Memory transfer for bitblt's     */
int     xres,yres;                  /* Video mode resolution            */
int     bytesperline;               /* Bytes per scanline for mode      */
long    imageSize;                  /* Length of the video image        */
char    *LFBPtr;                	/* Pointer to linear framebuffer    */

/*------------------------- DPMI interface routines -----------------------*/

void DPMI_allocRealSeg(int size,int *sel,int *r_seg)
/****************************************************************************
*
* Function:     DPMI_allocRealSeg
* Parameters:   size    - Size of memory block to allocate
*               sel     - Place to return protected mode selector
*               r_seg   - Place to return real mode segment
*
* Description:  Allocates a block of real mode memory using DPMI services.
*               This routine returns both a protected mode selector and
*               real mode segment for accessing the memory block.
*
****************************************************************************/
{
    union REGS      r;

    r.x.ax = 0x100;                 /* DPMI allocate DOS memory         */
    r.x.bx = (size + 0xF) >> 4;     /* number of paragraphs             */
    intdos(&r, &r);
    if (r.x.cflag);
    *sel = r.x.dx;                  /* Protected mode selector          */
    *r_seg = r.x.ax;                /* Real mode segment                */
}

void DPMI_freeRealSeg(unsigned sel)
/****************************************************************************
*
* Function:     DPMI_allocRealSeg
* Parameters:   sel - Protected mode selector of block to free
*
* Description:  Frees a block of real mode memory.
*
****************************************************************************/
{
    union REGS  r;

    r.x.ax = 0x101;                 /* DPMI free DOS memory             */
    r.x.dx = sel;                   /* DX := selector from 0x100        */
    intdos(&r, &r);
}

typedef struct {
    long    edi;
    long    esi;
    long    ebp;
    long    reserved;
    long    ebx;
    long    edx;
    long    ecx;
    long    eax;
    short   flags;
    short   es,ds,fs,gs,ip,cs,sp,ss;
    } _RMREGS;

#define IN(reg)     rmregs.e##reg = in->x.reg
#define OUT(reg)    out->x.reg = rmregs.e##reg

int DPMI_intdos(int intno, RMREGS *in, RMREGS *out)
/****************************************************************************
*
* Function:     DPMI_int86
* Parameters:   intno   - Interrupt number to issue
*               in      - Pointer to structure for input registers
*               out     - Pointer to structure for output registers
* Returns:      Value returned by interrupt in AX
*
* Description:  Issues a real mode interrupt using DPMI services.
*
****************************************************************************/
{
    _RMREGS         rmregs;
    union REGS      r;
    struct SREGS    sr;

    memset(&rmregs, 0, sizeof(rmregs));
    IN(ax); IN(bx); IN(cx); IN(dx); IN(si); IN(di);

    segread(&sr);
    r.x.ax = 0x300;                 /* DPMI issue real interrupt        */
    r.h.bl = intno;
    r.h.bh = 0;
    r.x.cx = 0;
    sr.es = sr.ds;
    r.x.di = (unsigned)&rmregs;
    intdosx(&r, &r, &sr);     /* Issue the interrupt              */

    OUT(ax); OUT(bx); OUT(cx); OUT(dx); OUT(si); OUT(di);
    out->x.cflag = rmregs.flags & 0x1;
    return out->x.ax;
}

int DPMI_intdosx(int intno, RMREGS *in, RMREGS *out, RMSREGS *sregs)
/****************************************************************************
*
* Function:     DPMI_int86
* Parameters:   intno   - Interrupt number to issue
*               in      - Pointer to structure for input registers
*               out     - Pointer to structure for output registers
*               sregs   - Values to load into segment registers
* Returns:      Value returned by interrupt in AX
*
* Description:  Issues a real mode interrupt using DPMI services.
*
****************************************************************************/
{
    _RMREGS         rmregs;
    union REGS      r;
    struct SREGS    sr;

    memset(&rmregs, 0, sizeof(rmregs));
    IN(ax); IN(bx); IN(cx); IN(dx); IN(si); IN(di);
    rmregs.es = sregs->es;
    rmregs.ds = sregs->ds;

    segread(&sr);
    r.x.ax = 0x300;                 /* DPMI issue real interrupt        */
    r.h.bl = intno;
    r.h.bh = 0;
    r.x.cx = 0;
    sr.es = sr.ds;
    r.x.di = (unsigned)&rmregs;
    intdosx(&r, &r, &sr);     /* Issue the interrupt */

    OUT(ax); OUT(bx); OUT(cx); OUT(dx); OUT(si); OUT(di);
    sregs->es = rmregs.es;
    sregs->cs = rmregs.cs;
    sregs->ss = rmregs.ss;
    sregs->ds = rmregs.ds;
    out->x.cflag = rmregs.flags & 0x1;
    return out->x.ax;
}

int DPMI_allocSelector(void)
/****************************************************************************
*
* Function:     DPMI_allocSelector
* Returns:      Newly allocated protected mode selector
*
* Description:  Allocates a new protected mode selector using DPMI
*               services. This selector has a base address and limit of 0.
*
****************************************************************************/
{
    int         sel;
    union REGS  r;

    r.x.ax = 0;                     /* DPMI allocate selector           */
    r.x.cx = 1;                     /* Allocate a single selector       */
    intdos(&r, &r);
    if (r.x.cflag);
    sel = r.x.ax;

    r.x.ax = 9;                     /* DPMI set access rights           */
    r.x.bx = sel;
    r.x.cx = 0x8092;                /* 32 bit page granular             */
    intdos(&r, &r);
    return sel;
}

long DPMI_mapPhysicalToLinear(long physAddr,long limit)
/****************************************************************************
*
* Function:     DPMI_mapPhysicalToLinear
* Parameters:   physAddr    - Physical memory address to map
*               limit       - Length-1 of physical memory region to map
* Returns:      Starting linear address for mapped memory
*
* Description:  Maps a section of physical memory into the linear address
*               space of a process using DPMI calls. Note that this linear
*               address cannot be used directly, but must be used as the
*               base address for a selector.
*
****************************************************************************/
{
    union REGS  r;

    r.x.ax = 0x800;                 /* DPMI map physical to linear      */
    r.x.bx = physAddr >> 16;
    r.x.cx = physAddr & 0xFFFF;
    r.x.si = limit >> 16;
    r.x.di = limit & 0xFFFF;
    intdos(&r, &r);
    if (r.x.cflag);
    return ((long)r.x.bx << 16) + r.x.cx;
}

void DPMI_setSelectorBase(int sel,long linAddr)
/****************************************************************************
*
* Function:     DPMI_setSelectorBase
* Parameters:   sel     - Selector to change base address for
*               linAddr - Linear address used for new base address
*
* Description:  Sets the base address for the specified selector.
*
****************************************************************************/
{
    union REGS  r;

    r.x.ax = 7;                     /* DPMI set selector base address   */
    r.x.bx = sel;
    r.x.cx = linAddr >> 16;
    r.x.dx = linAddr & 0xFFFF;
    intdos(&r, &r);
    if (r.x.cflag);
}

void DPMI_setSelectorLimit(int sel,long limit)
/****************************************************************************
*
* Function:     DPMI_setSelectorLimit
* Parameters:   sel     - Selector to change limit for
*               limit   - Limit-1 for the selector
*
* Description:  Sets the memory limit for the specified selector.
*
****************************************************************************/
{
    union REGS  r;

    r.x.ax = 8;                     /* DPMI set selector limit          */
    r.x.bx = sel;
    r.x.cx = limit >> 16;
    r.x.dx = limit & 0xFFFF;
    intdos(&r, &r);
    if (r.x.cflag);
}


static void ExitVBEBuf(void)
{
    DPMI_freeRealSeg(VESABuf_sel);
}

void VBE_initRMBuf(void)
/****************************************************************************
*
* Function:     VBE_initRMBuf
* Description:  Initialises the VBE transfer buffer in real mode memory.
*               This routine is called by the VESAVBE module every time
*               it needs to use the transfer buffer, so we simply allocate
*               it once and then return.
*
****************************************************************************/
{
    if (!VESABuf_sel) {
        DPMI_allocRealSeg(VESABuf_len, &VESABuf_sel, &VESABuf_rseg);
        atexit(ExitVBEBuf);
        }
}

void VBE_callESDI(RMREGS *regs, void *buffer, int size)
/****************************************************************************
*
* Function:     VBE_callESDI
* Parameters:   regs    - Registers to load when calling VBE
*               buffer  - Buffer to copy VBE info block to
*               size    - Size of buffer to fill
*
* Description:  Calls the VESA VBE and passes in a buffer for the VBE to
*               store information in, which is then copied into the users
*               buffer space. This works in protected mode as the buffer
*               passed to the VESA VBE is allocated in conventional
*               memory, and is then copied into the users memory block.
*
****************************************************************************/
{
    RMSREGS sregs;

    VBE_initRMBuf();
    sregs.es = VESABuf_rseg;
    regs->x.di = 0;
    memcpy((void *)MK_FP(VESABuf_sel,0),(const void *)buffer,(size_t)size);
    DPMI_intdosx(0x10, regs, regs, &sregs);
    memcpy((void *)buffer,(const void *)MK_FP(VESABuf_sel,0),(size_t)size);
}

int VBE_detect(void)
/****************************************************************************
*
* Function:     VBE_detect
* Parameters:   vgaInfo - Place to store the VGA information block
* Returns:      VBE version number, or 0 if not detected.
*
* Description:  Detects if a VESA VBE is out there and functioning
*               correctly. If we detect a VBE interface we return the
*               VGAInfoBlock returned by the VBE and the VBE version number.
*
****************************************************************************/
{
    RMREGS      regs;
    unsigned    short    *p1,*p2;
    VBE_vgaInfo vgaInfo;

    /* Put 'VBE2' into the signature area so that the VBE 2.0 BIOS knows
     * that we have passed a 512 byte extended block to it, and wish
     * the extended information to be filled in.
     */
    strncpy(vgaInfo.VESASignature,"VBE2",4);

    /* Get the SuperVGA Information block */
    regs.x.ax = 0x4F00;
    VBE_callESDI(&regs, &vgaInfo, sizeof(VBE_vgaInfo));
    if (regs.x.ax != 0x004F)
        return 0;
    if (strncmp(vgaInfo.VESASignature,"VESA",4) != 0)
        return 0;

    /* Now that we have detected a VBE interface, copy the list of available
     * video modes into our local buffer. We *must* copy this mode list,
     * since the VBE will build the mode list in the VBE_vgaInfo buffer
     * that we have passed, so the next call to the VBE will trash the
     * list of modes.
     */
    printf("videomodeptr %x\n",vgaInfo.VideoModePtr);
    p1 = LfbMapRealPointer(vgaInfo.VideoModePtr);
    p2 = modeList;
    while (*p1 != NULL)
    {
        printf("found mode %x\n",*p1);
        *p2++ = *p1++;
    }
    *p2 = -1;
    return vgaInfo.VESAVersion;
}

int VBE_getModeInfo(int mode,VBE_modeInfo *modeInfo)
/****************************************************************************
*
* Function:     VBE_getModeInfo
* Parameters:   mode        - VBE mode to get information for
*               modeInfo    - Place to store VBE mode information
* Returns:      1 on success, 0 if function failed.
*
* Description:  Obtains information about a specific video mode from the
*               VBE. You should use this function to find the video mode
*               you wish to set, as the new VBE 2.0 mode numbers may be
*               completely arbitrary.
*
****************************************************************************/
{
    RMREGS  regs;

    regs.x.ax = 0x4F01;             /* Get mode information         */
    regs.x.cx = mode;
    VBE_callESDI(&regs, modeInfo, sizeof(VBE_modeInfo));
    if (regs.x.ax != 0x004F)
        return 0;
    if ((modeInfo->ModeAttributes & vbeMdAvailable) == 0)
        return 0;
    return 1;
}

void VBE_setVideoMode(int mode)
/****************************************************************************
*
* Function:     VBE_setVideoMode
* Parameters:   mode    - VBE mode number to initialise
*
****************************************************************************/
{
    RMREGS  regs;
    regs.x.ax = 0x4F02;
    regs.x.bx = mode;
    DPMI_intdos(0x10,&regs,&regs);
}

/*-------------------- Application specific routines ----------------------*/

void *GetPtrToLFB(long physAddr)
/****************************************************************************
*
* Function:     GetPtrToLFB
* Parameters:   physAddr    - Physical memory address of linear framebuffer
* Returns:      Far pointer to the linear framebuffer memory
*
****************************************************************************/
{
    int     sel;
    long    linAddr,limit = (4096 * 1024) - 1;

	linAddr = DPMI_mapPhysicalToLinear(physAddr,limit);
	return (void*)linAddr;
}

void AvailableModes(void)
/****************************************************************************
*
* Function:     AvailableModes
*
* Description:  Display a list of available LFB mode resolutions.
*
****************************************************************************/
{
    unsigned short           *p;
    VBE_modeInfo    modeInfo;

    printf("Usage: LFBPROF <xres> <yres>\n\n");
    printf("Available 256 color video modes:\n");
    for (p = modeList; *p != NULL; p++) {
        if (VBE_getModeInfo(*p, &modeInfo)) {
            /* Filter out only 8 bit linear framebuffer modes */
            if ((modeInfo.ModeAttributes & vbeMdLinear) == 0)
                continue;
            if (modeInfo.MemoryModel != vbeMemPK
                || modeInfo.BitsPerPixel != 8
                || modeInfo.NumberOfPlanes != 1)
                continue;
            printf("    %4d x %4d %d bits per pixel\n",
                modeInfo.XResolution, modeInfo.YResolution,
                modeInfo.BitsPerPixel);
            }
        }
    exit(1);
}

void InitGraphics(int x,int y)
/****************************************************************************
*
* Function:     InitGraphics
* Parameters:   x,y - Requested video mode resolution
*
* Description:  Initialise the specified video mode. We search through
*               the list of available video modes for one that matches
*               the resolution and color depth are are looking for.
*
****************************************************************************/
{
    unsigned short           *p;
    VBE_modeInfo    modeInfo;
    printf("InitGraphics\n");

    for (p = modeList; *p != NULL; p++) {
        if (VBE_getModeInfo(*p, &modeInfo)) {
            /* Filter out only 8 bit linear framebuffer modes */
			printf("XResolution: %d, YResolution: %d\r\n", modeInfo.XResolution, modeInfo.YResolution);
            if ((modeInfo.ModeAttributes & vbeMdLinear) == 0)
                continue;
            if (modeInfo.MemoryModel != vbeMemPK
                || modeInfo.BitsPerPixel != 8
                || modeInfo.NumberOfPlanes != 1)
                continue;
            if (modeInfo.XResolution != x || modeInfo.YResolution != y)
                continue;
            xres = x;
            yres = y;
            bytesperline = modeInfo.BytesPerScanLine;
            imageSize = bytesperline * yres;
            VBE_setVideoMode(*p | vbeUseLFB);
            LFBPtr = GetPtrToLFB(modeInfo.PhysBasePtr);
            return;
            }
        }
    printf("Valid video mode not found\n");
    exit(1);
}

void EndGraphics(void)
/****************************************************************************
*
* Function:     EndGraphics
*
* Description:  Restores text mode.
*
****************************************************************************/
{
    RMREGS  regs;
    printf("EndGraphics\n");
    regs.x.ax = 0x3;
    DPMI_intdos(0x10, &regs, &regs);
}

void ProfileMode(void)
/****************************************************************************
*
* Function:     ProfileMode
*
* Description:  Profiles framebuffer performance for simple screen clearing
*               and for copying from system memory to video memory (BitBlt).
*               This routine thrashes the CPU cache by cycling through
*               enough system memory buffers to invalidate the entire
*               CPU external cache before re-using the first memory buffer
*               again.
*
****************************************************************************/
{
    int     i,numClears,numBlts,maxImages;
    long    startTicks,endTicks;
    void    *image[10],*dst;
    printf("ProfileMode\n");

    /* Profile screen clearing operation */
    startTicks = LfbGetTicks();
    numClears = 0;
    while ((LfbGetTicks() - startTicks) < 182)
		LfbMemset(LFBPtr,numClears++,imageSize);
	endTicks = LfbGetTicks();
	clearsPerSec = numClears / ((endTicks - startTicks) * 0.054925);
	clearsMbPerSec = (clearsPerSec * imageSize) / 1048576.0;

	/* Profile system memory to video memory copies */
	maxImages = ((512 * 1024U) / imageSize) + 2;
	for (i = 0; i < maxImages; i++) {
		image[i] = malloc(imageSize);
		if (image[i] == NULL)
		{}
		memset(image[i],i+1,imageSize);
		}
	startTicks = LfbGetTicks();
	numBlts = 0;
	while ((LfbGetTicks() - startTicks) < 182)
		LfbMemcpy(LFBPtr,image[numBlts++ % maxImages],imageSize);
    endTicks = LfbGetTicks();
    bitBltsPerSec = numBlts / ((endTicks - startTicks) * 0.054925);
    bitBltsMbPerSec = (bitBltsPerSec * imageSize) / 1048576.0;
}

void main(int argc, char *argv[])
{
    if (VBE_detect() < 0x200);
    if (argc != 3)
        AvailableModes();       /* Display available modes              */

    InitGraphics(atoi(argv[1]),atoi(argv[2]));  /* Start graphics       */
    ProfileMode();              /* Profile the video mode               */
    EndGraphics();              /* Restore text mode                    */

    printf("Profiling results for %dx%d 8 bits per pixel.\n",xres,yres);
    printf("%3.2f clears/s, %2.2f Mb/s\n", clearsPerSec, clearsMbPerSec);
    printf("%3.2f bitBlt/s, %2.2f Mb/s\n", bitBltsPerSec, bitBltsMbPerSec);
}