
typedef struct {
   int codewords;       /* -c, --codewords */
   int edgeMin;         /* -e, --minimum-edge */
   int edgeMax;         /* -E, --maximum-edge */
   int scanGap;         /* -g, --gap */
   int timeoutMS;       /* -m, --milliseconds */
   int newline;         /* -n, --newline */
   int page;            /* -p, --page */
   int squareDevn;      /* -q, --square-deviation */
   int dpi;             /* -r, --resolution */
   int sizeIdxExpected; /* -s, --symbol-size */
   int edgeThresh;      /* -t, --threshold */
   char *xMin;          /* -x, --x-range-min */
   char *xMax;          /* -X, --x-range-max */
   char *yMin;          /* -y, --y-range-min */
   char *yMax;          /* -Y, --y-range-max */
   int correctionsMax;  /* -C, --corrections-max */
   int diagnose;        /* -D, --diagnose */
   int mosaic;          /* -M, --mosaic */
   int stopAfter;       /* -N, --stop-after */
   int pageNumbers;     /* -P, --page-numbers */
   int corners;         /* -R, --corners */
   int shrinkMax;       /* -S, --shrink */
   int shrinkMin;       /* -S, --shrink (if range specified) */
   int unicode;         /* -U, --unicode */
   int gs1;             /* -G, --gs1 */
   int verbose;         /* -v, --verbose */
} UserOptions;

/* Functions */
static UserOptions GetDefaultOptions(void);
//static DmtxPassFail HandleArgs(UserOptions *opt, int *fileIndex, int *argcp, char **argvp[]);
//static void ShowUsage(int status);
static DmtxPassFail SetDecodeOptions(DmtxDecode *dec, DmtxImage *img, UserOptions *opt);
//static DmtxPassFail PrintStats(DmtxDecode *dec, DmtxRegion *reg, DmtxMessage *msg, int imgPageIndex, UserOptions *opt);
//static DmtxPassFail PrintMessage(DmtxRegion *reg, DmtxMessage *msg, UserOptions *opt);
//std::string codeToString(DmtxRegion *reg, DmtxMessage *msg, UserOptions *opt);
//static void ListImageFormats(void);
//static void WriteDiagnosticImage(DmtxDecode *dec, char *imagePath);
static int ScaleNumberString(char *s, int extent);




/**
 *
 *
 */
static DmtxPassFail
SetDecodeOptions(DmtxDecode *dec, DmtxImage *img, UserOptions *opt)
{
   int err;

#define RETURN_IF_FAILED(e) if(e != DmtxPass) { return DmtxFail; }

   err = dmtxDecodeSetProp(dec, DmtxPropScanGap, opt->scanGap);
   RETURN_IF_FAILED(err)

   if(opt->gs1 != DmtxUndefined) {
      //err = dmtxDecodeSetProp(dec, DmtxPropFnc1, opt->gs1);
      //RETURN_IF_FAILED(err)
   }

   if(opt->edgeMin != DmtxUndefined) {
      err = dmtxDecodeSetProp(dec, DmtxPropEdgeMin, opt->edgeMin);
      RETURN_IF_FAILED(err)
   }

   if(opt->edgeMax != DmtxUndefined) {
      err = dmtxDecodeSetProp(dec, DmtxPropEdgeMax, opt->edgeMax);
      RETURN_IF_FAILED(err)
   }

   if(opt->squareDevn != DmtxUndefined) {
      err = dmtxDecodeSetProp(dec, DmtxPropSquareDevn, opt->squareDevn);
      RETURN_IF_FAILED(err)
   }

   err = dmtxDecodeSetProp(dec, DmtxPropSymbolSize, opt->sizeIdxExpected);
   RETURN_IF_FAILED(err)

   err = dmtxDecodeSetProp(dec, DmtxPropEdgeThresh, opt->edgeThresh);
   RETURN_IF_FAILED(err)

   if(opt->xMin) {
      err = dmtxDecodeSetProp(dec, DmtxPropXmin, ScaleNumberString(opt->xMin, img->width));
      RETURN_IF_FAILED(err)
   }

   if(opt->xMax) {
      err = dmtxDecodeSetProp(dec, DmtxPropXmax, ScaleNumberString(opt->xMax, img->width));
      RETURN_IF_FAILED(err)
   }

   if(opt->yMin) {
      err = dmtxDecodeSetProp(dec, DmtxPropYmin, ScaleNumberString(opt->yMin, img->height));
      RETURN_IF_FAILED(err)
   }

   if(opt->yMax) {
      err = dmtxDecodeSetProp(dec, DmtxPropYmax, ScaleNumberString(opt->yMax, img->height));
      RETURN_IF_FAILED(err)
   }

#undef RETURN_IF_FAILED

   return DmtxPass;
}


/**
 *
 *
 */
static UserOptions
GetDefaultOptions(void)
{
   UserOptions opt;

   memset(&opt, 0x00, sizeof(UserOptions));

   /* Default options */
   opt.codewords = DmtxFalse;
   opt.edgeMin = DmtxUndefined;
   opt.edgeMax = DmtxUndefined;
   opt.scanGap = 2;
   opt.timeoutMS = 200; //DmtxUndefined;
   opt.newline = DmtxFalse;
   opt.page = DmtxUndefined;
   opt.squareDevn = DmtxUndefined;
   opt.dpi = DmtxUndefined;
   opt.sizeIdxExpected = DmtxSymbolShapeAuto;
   opt.edgeThresh = 5;
   opt.xMin = NULL;
   opt.xMax = NULL;
   opt.yMin = NULL;
   opt.yMax = NULL;
   opt.correctionsMax = DmtxUndefined;
   opt.diagnose = DmtxFalse;
   opt.mosaic = DmtxFalse;
   opt.stopAfter = DmtxUndefined;
   opt.pageNumbers = DmtxFalse;
   opt.corners = DmtxFalse;
   opt.shrinkMin = 1;
   opt.shrinkMax = 1;
   opt.unicode = DmtxFalse;
   opt.verbose = DmtxFalse;
   opt.gs1 = DmtxUndefined;

   return opt;
}



/**
 *
 *
 */
static int
ScaleNumberString(char *s, int extent)
{
  /*
   int numValue;
   int scaledValue;
   char *terminate;

   assert(s != NULL);

  numValue=atoi(s);
   err = StringToInt(&numValue, s, &terminate);
   if(err != DmtxPass){
      //FatalError(EX_USAGE, _("Integer value required"));
      return 0;
   }
   

   scaledValue = (*terminate == '%') ? (int)(0.01 * numValue * extent + 0.5) : numValue;

   if(scaledValue < 0)
      scaledValue = 0;

   if(scaledValue >= extent)
      scaledValue = extent - 1;
   */
   return 0;//scaledValue;
}



/*
static DmtxPassFail
PrintMessage(DmtxRegion *reg, DmtxMessage *msg, UserOptions *opt)
{
   int i;
   int remainingDataWords;
   int dataWordLength;
   

   if(opt->codewords == DmtxTrue) {
      dataWordLength = dmtxGetSymbolAttribute(DmtxSymAttribSymbolDataWords, reg->sizeIdx);
      for(i = 0; i < msg->codeSize; i++) {
         remainingDataWords = dataWordLength - i;
         if(remainingDataWords > msg->padCount)

            fprintf(stdout, "%c:%03d\n", 'd', msg->code[i]);
         else if(remainingDataWords > 0)
            fprintf(stdout, "%c:%03d\n", 'p', msg->code[i]);
         else
            fprintf(stdout, "%c:%03d\n", 'e', msg->code[i]);
      }
   }
   else {
      if(opt->unicode == DmtxTrue) {
         for(i = 0; i < msg->outputIdx; i++) {
            if(msg->output[i] < 128) {
               fputc(msg->output[i], stdout);
            }
            else if(msg->output[i] < 192) {
              fputc(0xc2, stdout);
              fputc(msg->output[i], stdout);
            }
            else {
               fputc(0xc3, stdout);
               fputc(msg->output[i] - 64, stdout);
            }
         }
      }
      else {
         fwrite(msg->output, sizeof(char), msg->outputIdx, stdout);
      }

      if(opt->newline)
         fputc('\n', stdout);
   }

   return DmtxPass;
}
*/
