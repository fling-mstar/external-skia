
/*
 * Copyright 2006 The Android Open Source Project
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */


#include "SkImageDecoder.h"
#include "SkImageEncoder.h"
#include "SkColor.h"
#include "SkColorPriv.h"
#include "SkDither.h"
#include "SkMath.h"
#include "SkRTConf.h"
#include "SkScaledBitmapSampler.h"
#include "SkStream.h"
#include "SkTemplates.h"
#include "SkUtils.h"
#include "transform_scanline.h"
extern "C" {
#include "png.h"
}

// MStar Android Patch Begin
#ifdef HW_PNG_SUPPORT
#include <mmap.h>
#include "MsTypes.h"
#include "MsCommon.h"
#include "MsOS.h"
#include "drvMMIO.h"
#include "drvMIU.h"
#include "drvSYS.h"
#include <utils/threads.h>
#define U8 MS_U8
#define U16 MS_U16
#define U32 MS_U32
#define S8 MS_S8
#define S16 MS_S16
#define S32 MS_S32

#include "apiGPD.h"
#include "apiGFX.h"
#include "drvSEM.h"
// GE convertion has limitation, max width is 4080
#define PHOTO_DECODE_PNG_MAX_WIDTH      (4000)
#define PHOTO_DECODE_PNG_MAX_HEIGHT     (4000)

#define PHOTO_DECODE_PNG_MIN_WIDTH      (256)
#define PHOTO_DECODE_PNG_MIN_HEIGHT     (256)

#define PNG_READ_BUFFER_SIZE      0x100000
#define PNG_INTERNAL_BUFFER_SIZE  0x100000
//#define TEST_TIME_USED   0

#include <cutils/properties.h>

static unsigned int read_paddr =0,read_len=0,inter_paddr=0,inter_len=0,write_paddr=0,write_len=0,final_paddr= 0,final_len=0;
static  void *pvaddr_read = NULL,*pvaddr_internal = NULL,*pvaddr_write = NULL,*pvaddr_final = NULL;
static  int s_index = -1;
#endif // HW_PNG_SUPPORT

#include <utils/Log.h>

#define ENABLE_PNG_DECODE_OPTIMIZE    1
#define PNG_NON_INTERLACE_MAX_WIDTH    (1920 * 8)
#define PNG_NON_INTERLACE_MAX_HEIGHT    (1080 * 8)

// The values below are the results of  many of tests.
#define PNG_INTERLACE_MAX_WIDTH    (7000)
#define PNG_INTERLACE_MAX_HEIGHT    (7000)
#define PNG_INTERLACE_MAX_WIDTH_AFTER_OPTIMIZE    (1920 * 8)
#define PNG_INTERLACE_MAX_HEIGHT_AFTER_OPTIMIZE    (1080 * 8)

// In optimize sw decoder,if the resolution of interlace png
// smaller than 4000*4000, go mode that read data with horizontal scale,
// else go mode that read data with horizontal and vertical scale.
#define PNG_INTERLACE_BORDER_WIDTH_IN_OPTIMIZE     (4000)
#define PNG_INTERLACE_BORDER_HEIGHT_IN_OPTIMIZE     (4000)
// MStar Android Patch End

/* These were dropped in libpng >= 1.4 */
#ifndef png_infopp_NULL
#define png_infopp_NULL NULL
#endif

#ifndef png_bytepp_NULL
#define png_bytepp_NULL NULL
#endif

#ifndef int_p_NULL
#define int_p_NULL NULL
#endif

#ifndef png_flush_ptr_NULL
#define png_flush_ptr_NULL NULL
#endif

#if defined(SK_DEBUG)
#define DEFAULT_FOR_SUPPRESS_PNG_IMAGE_DECODER_WARNINGS false
#else  // !defined(SK_DEBUG)
#define DEFAULT_FOR_SUPPRESS_PNG_IMAGE_DECODER_WARNINGS true
#endif  // defined(SK_DEBUG)
SK_CONF_DECLARE(bool, c_suppressPNGImageDecoderWarnings,
                "images.png.suppressDecoderWarnings",
                DEFAULT_FOR_SUPPRESS_PNG_IMAGE_DECODER_WARNINGS,
                "Suppress most PNG warnings when calling image decode "
                "functions.");



class SkPNGImageIndex {
public:
    SkPNGImageIndex(SkStreamRewindable* stream, png_structp png_ptr, png_infop info_ptr)
        : fStream(stream)
        , fPng_ptr(png_ptr)
        , fInfo_ptr(info_ptr)
        , fConfig(SkBitmap::kNo_Config) {
        SkASSERT(stream != NULL);
        stream->ref();
    }
    ~SkPNGImageIndex() {
        if (NULL != fPng_ptr) {
            png_destroy_read_struct(&fPng_ptr, &fInfo_ptr, png_infopp_NULL);
        }
    }

    SkAutoTUnref<SkStreamRewindable>    fStream;
    png_structp                         fPng_ptr;
    png_infop                           fInfo_ptr;
    SkBitmap::Config                    fConfig;
};

class SkPNGImageDecoder : public SkImageDecoder {
public:
    SkPNGImageDecoder() {
        fImageIndex = NULL;
    }
    virtual Format getFormat() const SK_OVERRIDE {
        return kPNG_Format;
    }

    virtual ~SkPNGImageDecoder() {
        SkDELETE(fImageIndex);
    }

protected:
#ifdef SK_BUILD_FOR_ANDROID
    virtual bool onBuildTileIndex(SkStreamRewindable *stream, int *width, int *height) SK_OVERRIDE;
    virtual bool onDecodeSubset(SkBitmap* bitmap, const SkIRect& region) SK_OVERRIDE;
#endif
    virtual bool onDecode(SkStream* stream, SkBitmap* bm, Mode) SK_OVERRIDE;

private:
    SkPNGImageIndex* fImageIndex;

    bool onDecodeInit(SkStream* stream, png_structp *png_ptrp, png_infop *info_ptrp);
    bool decodePalette(png_structp png_ptr, png_infop info_ptr,
                       bool * SK_RESTRICT hasAlphap, bool *reallyHasAlphap,
                       SkColorTable **colorTablep);
    bool getBitmapConfig(png_structp png_ptr, png_infop info_ptr,
                         SkBitmap::Config *config, bool *hasAlpha,
                         SkPMColor *theTranspColor);

    typedef SkImageDecoder INHERITED;
};

#ifndef png_jmpbuf
#  define png_jmpbuf(png_ptr) ((png_ptr)->jmpbuf)
#endif

#define PNG_BYTES_TO_CHECK 4

/* Automatically clean up after throwing an exception */
struct PNGAutoClean {
    PNGAutoClean(png_structp p, png_infop i): png_ptr(p), info_ptr(i) {}
    ~PNGAutoClean() {
        png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);
    }
private:
    png_structp png_ptr;
    png_infop info_ptr;
};

static void sk_read_fn(png_structp png_ptr, png_bytep data, png_size_t length) {
    SkStream* sk_stream = (SkStream*) png_get_io_ptr(png_ptr);
    size_t bytes = sk_stream->read(data, length);
    if (bytes != length) {
        png_error(png_ptr, "Read Error!");
    }
}

#ifdef SK_BUILD_FOR_ANDROID
static void sk_seek_fn(png_structp png_ptr, png_uint_32 offset) {
    SkStreamRewindable* sk_stream = (SkStreamRewindable*) png_get_io_ptr(png_ptr);
    if (!sk_stream->rewind()) {
        png_error(png_ptr, "Failed to rewind stream!");
    }
    (void)sk_stream->skip(offset);
}
#endif

static int sk_read_user_chunk(png_structp png_ptr, png_unknown_chunkp chunk) {
    SkImageDecoder::Peeker* peeker =
                    (SkImageDecoder::Peeker*)png_get_user_chunk_ptr(png_ptr);
    // peek() returning true means continue decoding
    return peeker->peek((const char*)chunk->name, chunk->data, chunk->size) ?
            1 : -1;
}

static void sk_error_fn(png_structp png_ptr, png_const_charp msg) {
    SkDEBUGF(("------ png error %s\n", msg));
    longjmp(png_jmpbuf(png_ptr), 1);
}

static void skip_src_rows(png_structp png_ptr, uint8_t storage[], int count) {
    for (int i = 0; i < count; i++) {
        uint8_t* tmp = storage;
        png_read_rows(png_ptr, &tmp, png_bytepp_NULL, 1);
    }
}

static bool pos_le(int value, int max) {
    return value > 0 && value <= max;
}

static bool substituteTranspColor(SkBitmap* bm, SkPMColor match) {
    SkASSERT(bm->config() == SkBitmap::kARGB_8888_Config);

    bool reallyHasAlpha = false;

    for (int y = bm->height() - 1; y >= 0; --y) {
        SkPMColor* p = bm->getAddr32(0, y);
        for (int x = bm->width() - 1; x >= 0; --x) {
            if (match == *p) {
                *p = 0;
                reallyHasAlpha = true;
            }
            p += 1;
        }
    }
    return reallyHasAlpha;
}

static bool canUpscalePaletteToConfig(SkBitmap::Config dstConfig,
                                      bool srcHasAlpha) {
    switch (dstConfig) {
        case SkBitmap::kARGB_8888_Config:
        case SkBitmap::kARGB_4444_Config:
            return true;
        case SkBitmap::kRGB_565_Config:
            // only return true if the src is opaque (since 565 is opaque)
            return !srcHasAlpha;
        default:
            return false;
    }
}

// call only if color_type is PALETTE. Returns true if the ctable has alpha
static bool hasTransparencyInPalette(png_structp png_ptr, png_infop info_ptr) {
    png_bytep trans;
    int num_trans;

    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
        png_get_tRNS(png_ptr, info_ptr, &trans, &num_trans, NULL);
        return num_trans > 0;
    }
    return false;
}

void do_nothing_warning_fn(png_structp, png_const_charp) {
    /* do nothing */
}

bool SkPNGImageDecoder::onDecodeInit(SkStream* sk_stream, png_structp *png_ptrp,
                                     png_infop *info_ptrp) {
    /* Create and initialize the png_struct with the desired error handler
    * functions.  If you want to use the default stderr and longjump method,
    * you can supply NULL for the last three parameters.  We also supply the
    * the compiler header file version, so that we know if the application
    * was compiled with a compatible version of the library.  */

    png_error_ptr user_warning_fn =
        (c_suppressPNGImageDecoderWarnings) ? (&do_nothing_warning_fn) : NULL;
    /* NULL means to leave as default library behavior. */
    /* c_suppressPNGImageDecoderWarnings default depends on SK_DEBUG. */
    /* To suppress warnings with a SK_DEBUG binary, set the
     * environment variable "skia_images_png_suppressDecoderWarnings"
     * to "true".  Inside a program that links to skia:
     * SK_CONF_SET("images.png.suppressDecoderWarnings", true); */

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING,
        NULL, sk_error_fn, user_warning_fn);
    //   png_voidp user_error_ptr, user_error_fn, user_warning_fn);
    if (png_ptr == NULL) {
        return false;
    }

    *png_ptrp = png_ptr;

    /* Allocate/initialize the memory for image information. */
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        png_destroy_read_struct(&png_ptr, png_infopp_NULL, png_infopp_NULL);
        return false;
    }
    *info_ptrp = info_ptr;

    /* Set error handling if you are using the setjmp/longjmp method (this is
    * the normal method of doing things with libpng).  REQUIRED unless you
    * set up your own error handlers in the png_create_read_struct() earlier.
    */
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);
        return false;
    }

    /* If you are using replacement read functions, instead of calling
    * png_init_io() here you would call:
    */
    png_set_read_fn(png_ptr, (void *)sk_stream, sk_read_fn);
#ifdef SK_BUILD_FOR_ANDROID
    png_set_seek_fn(png_ptr, sk_seek_fn);
#endif
    /* where user_io_ptr is a structure you want available to the callbacks */
    /* If we have already read some of the signature */
//  png_set_sig_bytes(png_ptr, 0 /* sig_read */ );

    // hookup our peeker so we can see any user-chunks the caller may be interested in
    png_set_keep_unknown_chunks(png_ptr, PNG_HANDLE_CHUNK_ALWAYS, (png_byte*)"", 0);
    if (this->getPeeker()) {
        png_set_read_user_chunk_fn(png_ptr, (png_voidp)this->getPeeker(), sk_read_user_chunk);
    }

    /* The call to png_read_info() gives us all of the information from the
    * PNG file before the first IDAT (image data chunk). */
    png_read_info(png_ptr, info_ptr);
    png_uint_32 origWidth, origHeight;
    int bitDepth, colorType;
    png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bitDepth,
                 &colorType, int_p_NULL, int_p_NULL, int_p_NULL);

    /* tell libpng to strip 16 bit/color files down to 8 bits/color */
    if (bitDepth == 16) {
        png_set_strip_16(png_ptr);
    }
    /* Extract multiple pixels with bit depths of 1, 2, and 4 from a single
     * byte into separate bytes (useful for paletted and grayscale images). */
    if (bitDepth < 8) {
        png_set_packing(png_ptr);
    }
    /* Expand grayscale images to the full 8 bits from 1, 2, or 4 bits/pixel */
    if (colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8) {
        png_set_expand_gray_1_2_4_to_8(png_ptr);
    }

    return true;
}

// MStar Android Patch Begin
#ifdef HW_PNG_SUPPORT
static int GetBytesPerPixelByFormat(GFX_Buffer_Format format)
{
    switch (format) {
        case GFX_FMT_YUV422:
        case GFX_FMT_RGB565:
            return 2;
        case GFX_FMT_ARGB8888:
        case GFX_FMT_ABGR8888:
            return 4;

        default:
            ALOGE("not support yet!!!");
            return -1;
    }

    return -1;
}

static bool GE_Convert_Format(int width,int height,GFX_Buffer_Format srcFormat,GFX_Buffer_Format destFormat,
                              unsigned long srcaddr, unsigned long destaddr, bool bpremultialpha)
{
    GFX_BufferInfo srcbuf, dstbuf;
    GFX_DrawRect bitbltInfo;
    GFX_Point v0, v1;
    GFX_Result  res;

    //premultiply

    if (bpremultialpha) {
        MApi_GFX_SetDFBBldFlags((MS_U16)GFX_DFB_BLD_FLAG_SRCPREMUL);
        MApi_GFX_SetDFBBldOP(GFX_DFB_BLD_OP_ONE, GFX_DFB_BLD_OP_ZERO);
    } else {
        MApi_GFX_SetDFBBldFlags((MS_U16)0x0);
        MApi_GFX_SetDFBBldOP(GFX_DFB_BLD_OP_ONE, GFX_DFB_BLD_OP_ZERO);
    }
    ALOGV("~!~,width = %d , height = %d  ,srcformat =%d, destformat = %d",width,height,srcFormat,destFormat);

    MApi_GFX_BeginDraw();

    MApi_GFX_EnableDFBBlending(TRUE);
    MApi_GFX_EnableAlphaBlending(TRUE);
    MApi_GFX_SetAlphaSrcFrom(ABL_FROM_ASRC);

    srcbuf.u32ColorFmt = srcFormat;
    srcbuf.u32Addr = srcaddr;
    srcbuf.u32Pitch = width * GetBytesPerPixelByFormat(srcFormat);
    srcbuf.u32Width = width;
    srcbuf.u32Height = height;
    res = MApi_GFX_SetSrcBufferInfo(&srcbuf, 0);
    if (res != GFX_SUCCESS) {
        ALOGE("GFX set SrcBuffer failed with res = %d \n",res);
        return FALSE;
    }

    if (srcFormat == GFX_FMT_YUV422) {
        //convert from YUV to ABGR
        MApi_GFX_SetDC_CSC_FMT(GFX_YUV_RGB2YUV_255, GFX_YUV_OUT_255, GFX_YUV_IN_255, GFX_YUV_YUYV,  GFX_YUV_YUYV);
    }

    dstbuf.u32ColorFmt = destFormat;
    dstbuf.u32Addr = destaddr;
    dstbuf.u32Pitch = width * GetBytesPerPixelByFormat(destFormat);
    dstbuf.u32Width = width;
    dstbuf.u32Height = height;

    res = MApi_GFX_SetDstBufferInfo(&dstbuf, 0) ;
    if (res != GFX_SUCCESS) {
        ALOGE("GFX set DetBuffer failed with res = %d \n",res);
        return FALSE;
    }

    bitbltInfo.srcblk.x = 0;
    bitbltInfo.srcblk.y = 0;
    bitbltInfo.srcblk.width = width;
    bitbltInfo.srcblk.height = height;

    bitbltInfo.dstblk.x = 0;
    bitbltInfo.dstblk.y = 0;
    bitbltInfo.dstblk.width = width;
    bitbltInfo.dstblk.height = height;


    v0.x = bitbltInfo.dstblk.x;
    v0.y = bitbltInfo.dstblk.y;

    v1.x = bitbltInfo.dstblk.x + bitbltInfo.dstblk.width;
    v1.y = bitbltInfo.dstblk.y + bitbltInfo.dstblk.height;

    MApi_GFX_SetClip(&v0, &v1);
    if (MApi_GFX_BitBlt(&bitbltInfo, GFXDRAW_FLAG_DEFAULT) != GFX_SUCCESS) {
        ALOGE("GFX BitBlt failed\n");
        return FALSE;
    }
    if (MApi_GFX_FlushQueue() != GFX_SUCCESS) {
        ALOGE("GFX FlushQueue failed\n");
        return FALSE;
    }

    MApi_GFX_EndDraw();

    return TRUE;
}

static unsigned int Alignment(unsigned int p, unsigned int alignment)
{
    return((p + (alignment - 1)) & ~(alignment - 1));
}

#endif // HW_PNG_SUPPORT

#ifdef TEST_TIME_USED
static struct timeval tv;
static struct timeval tv2;
#endif

#ifdef DUMP_RAW_IMG
    #include <fcntl.h>
int pic_buffer_dump(unsigned char *buf, int width, int height, int pitch,
                    const char *directory,
                    const char *prefix, SkBitmap::Config config)
{
    int num  = -1;
    int fd_p = -1;
    int fd_g = -1;
    int i, n;
    int len = (directory ? strlen(directory) : 0) + (prefix ? strlen(prefix) : 0) + 40;
    char filename[len];
    char head[30];
    bool rgb   = true;
    bool alpha = (config == SkBitmap::kARGB_8888_Config);

    if (config != SkBitmap::kARGB_8888_Config && config != SkBitmap::kRGB_565_Config)
        return -1;

    if (prefix) {
        // Find the lowest unused index.
        while (++num < 10000) {
            snprintf(filename, len, "%s/%s_%04d.ppm", directory, prefix, num);

            if (access(filename, F_OK) != 0) {
                snprintf(filename, len, "%s/%s_%04d.pgm", directory, prefix, num);

                if (access(filename, F_OK) != 0)
                    break;
            }
        }

        if (num == 10000) {
            return -1;
        }
    }

    // Create a file with the found index.
    if (rgb) {
        if (prefix)
            snprintf(filename, len, "%s/%s_%04d.ppm", directory, prefix, num);
        else
            snprintf(filename, len, "%s.ppm", directory);

        fd_p = open(filename,  O_EXCL|O_CREAT | O_WRONLY, 0644);
        if (fd_p < 0) {
            return -1;
        }
    }

    // Create a graymap for the alpha channel using the found index.
    if (alpha) {
        if (prefix)
            snprintf(filename, len, "%s/%s_%04d.pgm", directory, prefix, num);
        else
            snprintf(filename, len, "%s.pgm", directory);

        fd_g = open(filename, O_EXCL|O_CREAT | O_WRONLY, 0644);
        if (fd_g < 0) {
            close(fd_p);

            return -1;
        }
    }



    if (rgb) {
        // Write the pixmap header.
        snprintf(head, 30, "P6\n%d %d\n255\n", width, height );
        write(fd_p, head, strlen(head));

    }

    // Write the graymap header.
    if (alpha) {
        snprintf(head, 30, "P5\n%d %d\n255\n", width, height);

        write(fd_g, head, strlen(head));

    }
    // Write the pixmap (and graymap) data.
    for (i = 0; i < height; i++) {
        int n3;

        // Prepare one row.
        unsigned char *src8 = buf+i*pitch;

        // Write color buffer to pixmap file.
        if (rgb) {
            unsigned char buf_p[width * 3];

            if (config == SkBitmap::kARGB_8888_Config) {
                int k;
                for (k = 0; k < width; k++) {
                    buf_p[k*3+0] = src8[k*4];
                    buf_p[k*3+1] = src8[k*4+1];
                    buf_p[k*3+2] = src8[k*4+2];
                }
            } else {
                int k;
                unsigned short *src16 = (unsigned short *)src8;
                SkASSERT(((int)src16&1)==0);
                for (k = 0; k < width; k++) {
                    buf_p[k*3+0] = (src16[k] & 0xF800) >> 8;
                    buf_p[k*3+1] = (src16[k] & 0x07E0) >> 3;
                    buf_p[k*3+2] = (src16[k] & 0x001F) << 3;
                }
            }
            write(fd_p, buf_p, width * 3);
        }

        // Write alpha buffer to graymap file.
        if (alpha) {
            unsigned char buf_g[width];

            if (config == SkBitmap::kARGB_8888_Config) {
                int k;
                for (k = 0; k < width; k++) {
                    buf_g[k] = src8[k*4+3];
                }
            }

            write(fd_g, buf_g, width);
        }
    }

    // Close pixmap file.
    if (rgb)
        close(fd_p);

    // Close graymap file.
    if (alpha)
        close(fd_g);

    return 0;
}
#endif
// MStar Android Patch End

bool SkPNGImageDecoder::onDecode(SkStream* sk_stream, SkBitmap* decodedBitmap,
                                 Mode mode) {
    // MStar Android Patch Begin
#ifdef TEST_TIME_USED
    gettimeofday (&tv , 0);
#endif

#ifdef HW_PNG_SUPPORT
    bool ret = false;
    bool mutex_locked = false;
    bool bAllocMemoFail = false;

    if (s_index < 0) {
        s_index = MsOS_CreateNamedMutex((MS_S8 *)"SkiaDecodeMutex");
        ALOGV("png mutex index = %d ",s_index);
        if (s_index < 0) {
            ALOGE("create named mutex SkiaDecodeMutex failed.");
            return false;
        }
    }

    //LOGV("~!~ in HW_PNG_SUPPORT2 , pid = %d, filename=%s \n",getpid(),sk_stream->getFileName());
    do {
        png_structp png_ptr;
        png_infop info_ptr;
        png_uint_32 origWidth, origHeight;
        char value1[PROPERTY_VALUE_MAX],value2[PROPERTY_VALUE_MAX];

        property_get("mstar.disableHW_PNG", value1, "0");
        int disablePNGHW = atoi(value1);

        if (disablePNGHW || !bEnableHwDecode)
            break;

        // png hw does not support scaling down. go sw path.
        if (this->getSampleSize() != 1) {
            ALOGV("sample size:%d", this->getSampleSize());
            break;
        }

        if (onDecodeInit(sk_stream, &png_ptr, &info_ptr) == false) {
            ALOGE("onDecodeInit failed");
            break;
        }

        if (setjmp(png_jmpbuf(png_ptr))) {
            ALOGE("setjmp failed");
            break;
        }

        PNGAutoClean autoClean(png_ptr, info_ptr);

        int bit_depth, color_type, interlace_type;
        png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bit_depth,
                     &color_type, &interlace_type, int_p_NULL, int_p_NULL);


        unsigned int width,height;
        property_get("mstar.png_width", value1, "256");
        property_get("mstar.png_height", value2, "256");
        width = atoi(value1);
        height = atoi(value2);

        if (origWidth *origHeight < width *height) {
            // for png files less than 256*256, go SW decode
            //LOGV("~!~ org %d %d less than width*height %d*%d, just bypass \n",origWidth ,origHeight, width ,height);
            break;
        }
        if ((origWidth > PHOTO_DECODE_PNG_MAX_WIDTH) || (origHeight > PHOTO_DECODE_PNG_MAX_HEIGHT)) {
            ALOGI("over the limited png hw decode size! \n");
            break;
        }

        SkBitmap::Config    config;
        bool                hasAlpha = false;
        bool                doDither = this->getDitherImage();
        SkPMColor           theTranspColor = 0; // 0 tells us not to try to match
        if (getBitmapConfig(png_ptr, info_ptr, &config, &hasAlpha, &theTranspColor) == false) {
            break;
        }

        //LOGI("config = %d \n",config);
        if (config != SkBitmap::kARGB_8888_Config && config != SkBitmap::kRGB_565_Config) {
            break;
        }
        if (!MsOS_LockMutex(s_index, -1)) {
            if (origWidth*origHeight<=4*width*height) {
                ALOGI("hw png concurrency between threads, png (%dx%d) will go through sw decode mode!",
                     (int)origWidth, (int)origHeight);
                break;
            }
            MsOS_LockMutex(s_index, 0);
        }

        mutex_locked = true;

        if (!read_paddr && !inter_paddr && !write_paddr) {
            const mmap_info_t* minfo = NULL;

            if (mmap_init() != 0) {
                break;
            }
            //MsOS_MPool_SetDbgLevel(E_MsOSMPool_DBG_L1);

            if (!MDrv_SYS_GlobalInit())
                break;

            GFX_Config gfx_config;
            gfx_config.u8Miu = 0;
            gfx_config.u8Dither = FALSE;
            gfx_config.u32VCmdQSize = 0;
            gfx_config.u32VCmdQAddr = 0;
            gfx_config.bIsHK = 1;
            gfx_config.bIsCompt = FALSE;
            MApi_GFX_Init(&gfx_config);
            //LOGD("GE Init done\n");


            // the area in mmap locate in miu0
            minfo = mmap_get_info("E_DFB_JPD_READ");
            if (minfo == NULL) {
                ALOGE("mmap_get_info E_DFB_JPD_READ fail.");
            }

            if (minfo->size != 0) {
                // non cache set last param to 1
                if (!MsOS_MPool_Mapping(0, minfo->addr, minfo->size, 1)) {
                    ALOGE("mapping read buf failed, 0x%lx,0x%lx ", minfo->addr, minfo->size);
                    break;
                }
                read_paddr = minfo->addr;
                read_len = minfo->size;

            }

            minfo = mmap_get_info("E_DFB_JPD_WRITE");
            if (minfo == NULL) {
                ALOGE("mmap_get_info E_DFB_JPD_WRITE fail.");
                break;
            }
            // break if the length in mmap is ZERO
            if (minfo->size == 0)
                break;
            // non cache set last param to 1
            if (!MsOS_MPool_Mapping(0, minfo->addr, minfo->size, 1)) {
                ALOGE("mapping write buf failed, 0x%lx,0x%lx ", minfo->addr, minfo->size);
                break;
            }

            if (read_paddr == 0) {
                // for liteAndrod, only write buffer is nonzero. Divide E_DFB_JPD_WRITE into read/internal/write buffer
                if (minfo->size < (PNG_READ_BUFFER_SIZE + PNG_INTERNAL_BUFFER_SIZE)) {
                    ALOGE("write buffer is too small, 0x%lx ", minfo->size);
                    break;
                }
                read_paddr = minfo->addr;
                read_len = PNG_READ_BUFFER_SIZE;
                inter_paddr = minfo->addr + PNG_READ_BUFFER_SIZE;
                inter_len = PNG_INTERNAL_BUFFER_SIZE;
                write_paddr = minfo->addr + PNG_READ_BUFFER_SIZE + PNG_INTERNAL_BUFFER_SIZE;
                write_len = minfo->size - PNG_READ_BUFFER_SIZE - PNG_INTERNAL_BUFFER_SIZE;

            } else {
                if (minfo->size < PNG_INTERNAL_BUFFER_SIZE) {
                    ALOGE("write buffer is too small, 0x%lx ", minfo->size);
                    break;
                }
                inter_paddr = minfo->addr;
                inter_len = PNG_INTERNAL_BUFFER_SIZE;
                write_paddr = minfo->addr + PNG_INTERNAL_BUFFER_SIZE;
                write_len = minfo->size - PNG_INTERNAL_BUFFER_SIZE;
            }
            final_paddr = inter_paddr;
            final_len = inter_len + write_len;

            ALOGD("[skia png]: readbuf addr:0x%x, size: 0x%x\n write buff addr:0x%x,  size: 0x%x\n internal buff addr:0x%x,   size: 0x%x\nGE dst buff addr:0x%x,  size: 0x%x\n",
            read_paddr, read_len, write_paddr, write_len, inter_paddr, inter_len, final_paddr, final_len);
            // non cache to call MsOS_MPool_PA2KSEG1
            pvaddr_read = (void *)MsOS_MPool_PA2KSEG1(read_paddr);
            pvaddr_internal = (void *)MsOS_MPool_PA2KSEG1(inter_paddr);
            pvaddr_write = (void *)MsOS_MPool_PA2KSEG1(write_paddr);
            pvaddr_final = (void *)MsOS_MPool_PA2KSEG1(final_paddr);

            if (!pvaddr_read || !pvaddr_internal || !pvaddr_write) {
                ALOGE("get vaddr failed:pvaddr_read=%p,pvaddr_internal=%p,pvaddr_write=%p",pvaddr_read,pvaddr_internal,pvaddr_write);
                break;
            }
        }

        if (!pvaddr_read || !pvaddr_internal || !pvaddr_write) {
            ALOGE("!!!!!!get vaddr failed:pvaddr_read=%p,pvaddr_internal=%p,pvaddr_write=%p",pvaddr_read,pvaddr_internal,pvaddr_write);
            break;
        }

        if (sk_stream->getLength() > read_len) {
            ALOGI("the stream's length %d over 0x0000100000 , go with SW decode \n",sk_stream->getLength());
            break;
        }

        sk_stream->rewind();

        int filelength = sk_stream->getLength();
        if (filelength == 0) {
            ALOGE("filelength 0 ");
            break;
        }

        size_t bytes_read = 0;
        size_t position_read = 0;
        unsigned int actualLength = 0;
        bool goSw = false;

        do {
            position_read += bytes_read;
            actualLength = sk_stream->getLength();
            if ((position_read + actualLength) > read_len) {
                goSw = true;
                break;
            }

            bytes_read = sk_stream->read(pvaddr_read + position_read, actualLength);
            usleep(100);
        } while( bytes_read > 0);

        if (goSw)
            break;

        /*
        if ((int)bytes_read != filelength) {
            ALOGE("read error! bytes_read =%d, filelength =%d \n",bytes_read ,filelength);
            break;
        }
        */

        //LOGV("~!~ 3, filelength= %d,origWidth = %d,origHeight=%d",filelength,origWidth,origHeight);

        unsigned int pitch, output_length,aligned_width;

        aligned_width = Alignment(origWidth,2);

        GFX_Buffer_Format srcFormat,destFormat;
        bool dopremultiplyalpha = true;
        unsigned int colorspace;
        if (config == SkBitmap::kARGB_8888_Config) {
            srcFormat = GFX_FMT_ARGB8888;
            destFormat = GFX_FMT_ABGR8888;
            colorspace = ARGB8888;
            dopremultiplyalpha = true;
        } else if (config == SkBitmap::kRGB_565_Config) {
            srcFormat = GFX_FMT_RGB565;
            destFormat = GFX_FMT_RGB565;
            colorspace = RGB565;
            dopremultiplyalpha = false;
        } else {
            ALOGE("config is wrong.");
            break;
        }

        const int sampleSize = this->getSampleSize();
        SkScaledBitmapSampler sampler(origWidth, origHeight, sampleSize);

        decodedBitmap->lockPixels();
        void* rowptr = (void*) decodedBitmap->getPixels();
        bool reuseBitmap = (rowptr != NULL);
        decodedBitmap->unlockPixels();

        //LOGI("org width %d,height %d :  decode width %d,height %d \n",sampler.scaledWidth(),sampler.scaledHeight(),decodedBitmap->width(),decodedBitmap->height());

        if (reuseBitmap && (sampler.scaledWidth() != decodedBitmap->width() ||
                            sampler.scaledHeight() != decodedBitmap->height())) {
            // Dimensions must match
            ALOGI("Dimensions not match");
            break;
        }
        //LOGI("~!~! config = %d, reuseBitmap = %d,hasAlpha = %d,doDither = %d,theTranspColor=%d \n",(int)config,reuseBitmap,hasAlpha,doDither,theTranspColor);

        if (!reuseBitmap) {
            //config = SkBitmap::kARGB_8888_Config; // force to set config to ARGB8888
            decodedBitmap->setConfig(config, origWidth,
                                     origHeight, 0);
        }
        if (SkImageDecoder::kDecodeBounds_Mode == mode) {
            ret = true;
            break;
        }

        if (!reuseBitmap) {
            if (!this->allocPixelRef(decodedBitmap, NULL)) {
                ALOGE("allocPixelRef fail");
                bAllocMemoFail = true;
                break;
            }
        }

        int bytesperpixel = GetBytesPerPixelByFormat(destFormat);
        gpd_pic_info pic_info;
        int res;
        gpd_roi_info roi;
        int DecodeHeight, ProcessedHeight = 0, unProcessedHeight, CanDecodeMaxLine;
        bool bFinished = false;
        char*ptr;
        void *psrcbuf;

        MsOS_FlushMemory();

        // non cache set param to 0
        if (!MApi_GPD_SetControl(E_GPD_USER_CMD_SET_CACHEABLE,(MS_U32)0)) {
            ALOGE("setcontrol read buf failed, 0x%x ",read_paddr);
            break;
        }
        gpd_access_region stAccessCfg;
        stAccessCfg.u32PA_StartAddr = (MS_U32)read_paddr;
        stAccessCfg.u32PA_EndAddr = (MS_U32)(write_paddr + write_len);
        MApi_GPD_SetControl(E_GPD_USER_CMD_SET_ACCESS_REGION, (MS_U32)&stAccessCfg);

        // call HW routine to decode
        MApi_GPD_Init(inter_paddr);

        res = MApi_GPD_InputSource(&pic_info, read_paddr, position_read);
        if (res < 0) {
            ALOGE("[png]:MApi_GPD_InputSource() return %d", res);
            break;
        }

        CanDecodeMaxLine = (write_len/(aligned_width*bytesperpixel))& 0xFFFFFFF8;
        unProcessedHeight = origHeight;

        do {
            if (unProcessedHeight > CanDecodeMaxLine) {
                DecodeHeight = CanDecodeMaxLine;
            } else {
                DecodeHeight = unProcessedHeight;
            }
            roi.hstart = 0;
            roi.vstart = ProcessedHeight;
            roi.width = origWidth;
            roi.height = DecodeHeight;

            //LOGD("png Decode ROI:%d,%d,%d,%d", roi.hstart, roi.vstart, roi.width, roi.height);
            res = MApi_GPD_OutputDecodeROI(write_paddr, colorspace, aligned_width * DecodeHeight * bytesperpixel, &roi);

            if (res != 0) {
                ALOGE("hw decode error: 0x%lx, 0x%lx, res=%d\n", pic_info.u32Width,pic_info.u32Height,res);
                break;
            } else {
                ALOGI("png decode success\n");
            }
            psrcbuf = pvaddr_write;

            // do not set opaque information???~!~
            if (config == SkBitmap::kARGB_8888_Config) {
                psrcbuf = pvaddr_final;
                GE_Convert_Format(aligned_width,DecodeHeight,srcFormat,destFormat,
                                  write_paddr,inter_paddr,dopremultiplyalpha&&(!bDisablePreMultiplyAlpha));
            }
            //decodedBitmap->setPixels(pvaddr_write, NULL);
            decodedBitmap->lockPixels();
            ptr = (char *)decodedBitmap->getPixels();
            ptr += (origWidth*ProcessedHeight*bytesperpixel);
            if (origWidth != aligned_width) {
                int i;
                for (i = 0 ; i < DecodeHeight; i ++) {
                    memcpy((ptr+i*origWidth*bytesperpixel),((char *)psrcbuf+i*aligned_width*bytesperpixel),origWidth*bytesperpixel);
                }
            } else {
                memcpy(ptr,psrcbuf, origWidth*DecodeHeight*bytesperpixel);
            }

            decodedBitmap->unlockPixels();

#ifdef DUMP_RAW_IMG
            pic_buffer_dump((unsigned char *)decodedBitmap->getPixels(), decodedBitmap->width(), decodedBitmap->height(),
                            decodedBitmap->rowBytes(), "/mnt/usb/sda1", "dump_hw_png", decodedBitmap->config());
#endif
            unProcessedHeight -= DecodeHeight;
            ProcessedHeight += DecodeHeight;
            if (ProcessedHeight >= (int)origHeight) {
                bFinished = true;
            } else {
                MApi_GPD_Init(inter_paddr);
            }
        } while (!bFinished);
        if (reuseBitmap)
            decodedBitmap->notifyPixelsChanged();

        //decodedBitmap->setIsOpaque(0);
        if (bFinished) {
            ret = true;
        }

#ifdef TEST_TIME_USED
        gettimeofday (&tv2 , 0);
        ALOGD("~!~!~ orgwidth=%d,orgHeight = %d, png HW decode end at:%ld ......\n",origWidth, origHeight,((tv2.tv_sec-tv.tv_sec)*1000+(tv2.tv_usec-tv.tv_usec)/1000));
#endif
    } while (0);

    if (mutex_locked)
        MsOS_UnlockMutex(s_index, 0);

    if (ret)
        return true;
    if (bAllocMemoFail)
        return false;

    // ALOGD("[png]: go to sw decode");
    sk_stream->rewind();

#endif // HW_PNG_SUPPORT
    // MStar Android Patch End
    png_structp png_ptr;
    png_infop info_ptr;

    if (!onDecodeInit(sk_stream, &png_ptr, &info_ptr)) {
        return false;
    }

    PNGAutoClean autoClean(png_ptr, info_ptr);

    if (setjmp(png_jmpbuf(png_ptr))) {
        return false;
    }

    png_uint_32 origWidth, origHeight;
    int bitDepth, colorType, interlaceType;
    png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bitDepth,
                 &colorType, &interlaceType, int_p_NULL, int_p_NULL);

    SkBitmap::Config    config;
    bool                hasAlpha = false;
    SkPMColor           theTranspColor = 0; // 0 tells us not to try to match

    if (!this->getBitmapConfig(png_ptr, info_ptr, &config, &hasAlpha, &theTranspColor)) {
        return false;
    }

    const int sampleSize = this->getSampleSize();
    SkScaledBitmapSampler sampler(origWidth, origHeight, sampleSize);
    decodedBitmap->setConfig(config, sampler.scaledWidth(), sampler.scaledHeight());

    // we should communicate alphaType, even if we early-return in bounds-only-mode.
    if (this->getRequireUnpremultipliedColors()) {
        decodedBitmap->setAlphaType(kUnpremul_SkAlphaType);
    }

    if (SkImageDecoder::kDecodeBounds_Mode == mode) {
        return true;
    }

    // MStar Android Patch Begin
    if (PNG_INTERLACE_NONE != interlaceType) {
#if ENABLE_PNG_DECODE_OPTIMIZE
        if ((origWidth * origHeight) > (PNG_INTERLACE_MAX_WIDTH_AFTER_OPTIMIZE * PNG_INTERLACE_MAX_HEIGHT_AFTER_OPTIMIZE)) {
            return false;
        }
#else
        if ((origWidth * origHeight) > (PNG_INTERLACE_MAX_WIDTH * PNG_INTERLACE_MAX_HEIGHT)) {
            return false;
        }
#endif
    } else {
        if ((origWidth * origHeight) > (PNG_NON_INTERLACE_MAX_WIDTH * PNG_NON_INTERLACE_MAX_HEIGHT)) {
            return false;
        }
    }
    // MStar Android Patch End

    // from here down we are concerned with colortables and pixels

    // we track if we actually see a non-opaque pixels, since sometimes a PNG sets its colortype
    // to |= PNG_COLOR_MASK_ALPHA, but all of its pixels are in fact opaque. We care, since we
    // draw lots faster if we can flag the bitmap has being opaque
    bool reallyHasAlpha = false;
    SkColorTable* colorTable = NULL;

    if (colorType == PNG_COLOR_TYPE_PALETTE) {
        decodePalette(png_ptr, info_ptr, &hasAlpha, &reallyHasAlpha, &colorTable);
    }

    SkAutoUnref aur(colorTable);

    if (!this->allocPixelRef(decodedBitmap,
                             SkBitmap::kIndex8_Config == config ? colorTable : NULL)) {
        return false;
    }

    SkAutoLockPixels alp(*decodedBitmap);

    /* Turn on interlace handling.  REQUIRED if you are not using
    *  png_read_image().  To see how to handle interlacing passes,
    *  see the png_read_row() method below:
    */
    const int number_passes = (interlaceType != PNG_INTERLACE_NONE) ?
                              png_set_interlace_handling(png_ptr) : 1;

    /* Optional call to gamma correct and add the background to the palette
    *  and update info structure.  REQUIRED if you are expecting libpng to
    *  update the palette for you (ie you selected such a transform above).
    */
    png_read_update_info(png_ptr, info_ptr);

    if ((SkBitmap::kA8_Config == config || SkBitmap::kIndex8_Config == config)
        && 1 == sampleSize) {
        if (SkBitmap::kA8_Config == config) {
            // For an A8 bitmap, we assume there is an alpha for speed. It is
            // possible the bitmap is opaque, but that is an unlikely use case
            // since it would not be very interesting.
            reallyHasAlpha = true;
            // A8 is only allowed if the original was GRAY.
            SkASSERT(PNG_COLOR_TYPE_GRAY == colorType);
        }
        for (int i = 0; i < number_passes; i++) {
            for (png_uint_32 y = 0; y < origHeight; y++) {
                uint8_t* bmRow = decodedBitmap->getAddr8(0, y);
                png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
            }
        }
    } else {
        SkScaledBitmapSampler::SrcConfig sc;
        int srcBytesPerPixel = 4;

        if (colorTable != NULL) {
            sc = SkScaledBitmapSampler::kIndex;
            srcBytesPerPixel = 1;
        } else if (SkBitmap::kA8_Config == config) {
            // A8 is only allowed if the original was GRAY.
            SkASSERT(PNG_COLOR_TYPE_GRAY == colorType);
            sc = SkScaledBitmapSampler::kGray;
            srcBytesPerPixel = 1;
        } else if (hasAlpha) {
            sc = SkScaledBitmapSampler::kRGBA;
        } else {
            sc = SkScaledBitmapSampler::kRGBX;
        }

        /*  We have to pass the colortable explicitly, since we may have one
            even if our decodedBitmap doesn't, due to the request that we
            upscale png's palette to a direct model
         */
        SkAutoLockColors ctLock(colorTable);
        // MStar Android Patch Begin
        if (bDisablePreMultiplyAlpha) {
            bool doDither = this->getDitherImage();
            if (!sampler.beginWithoutPremultiplyAlpha(decodedBitmap, sc, doDither, ctLock.colors())) {
                return false;
            }
        } else {
             if (!sampler.begin(decodedBitmap, sc, *this, ctLock.colors())) {
                return false;
            }
        }
        const int height = decodedBitmap->height();
        const int width = decodedBitmap->width();
        // MStar Android Patch End

        if (number_passes > 1) {
            // MStar Android Patch Begin
#if ENABLE_PNG_DECODE_OPTIMIZE
            // In optimize sw decoder,if the resolution of interlace png
            // smaller than 4000*4000, go mode that read data with horizontal scale,
            // else go mode that read data with horizontal and vertical scale.
            if ((origWidth * origHeight) <= (PNG_INTERLACE_BORDER_WIDTH_IN_OPTIMIZE * PNG_INTERLACE_BORDER_HEIGHT_IN_OPTIMIZE)) {
                // ALOGD("------[png]: go mode that read data with horizontal scale");
                SkAutoMalloc storage(origWidth * height * srcBytesPerPixel);
                uint8_t* base = (uint8_t*)storage.get();
                size_t rb = origWidth * srcBytesPerPixel;
                png_uint_32 startHeight = origHeight - sampler.srcY0();

                uint8_t* tempRow = (uint8_t*)malloc(origWidth * srcBytesPerPixel);
                int rowNumber = 0;

                for (int i = 0; i < number_passes; i++) {
                    uint8_t* row = base;
                    skip_src_rows(png_ptr, row, sampler.srcY0());
                    row = base;

                    rowNumber = 0;
                    for (png_uint_32 y = 0; y < startHeight; y++) {
                        uint8_t* bmRow = row;
                        png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
                        if (((y % sampler.srcDY()) == 0) && (rowNumber < (height - 1))) {
                            row += rb;
                            rowNumber++;
                        } else if (((y % sampler.srcDY()) == 0) && (rowNumber >= (height - 1))) {
                            row = tempRow;
                        }
                    }
                }

                free(tempRow);

                // now sample it
                for (int y = 0; y < height; y++) {
                    reallyHasAlpha |= sampler.next(base);
                    base += rb;
                }
            } else {
                // ALOGD("------[png]: go mode that read data with horizontal and vertical scale");
                uint8_t* origRow = (uint8_t*)malloc(origWidth * srcBytesPerPixel);
                size_t rowStart = sampler.srcX0() * srcBytesPerPixel;
                size_t deltaSrc = sampler.srcDX() * srcBytesPerPixel;

                SkAutoMalloc storage(width * height * srcBytesPerPixel);
                uint8_t* base = (uint8_t*)storage.get();
                size_t rb = width * srcBytesPerPixel;
                png_uint_32 startHeight = origHeight - sampler.srcY0();
                for (int i = 0; i < number_passes; i++) {
                    uint8_t* row = base;
                    uint8_t* origTemp = origRow;

                    skip_src_rows(png_ptr, origTemp, sampler.srcY0());

                    for (png_uint_32 y = 0; y < startHeight; y++) {
                        uint8_t* bmRow = row;

                        if ((y % sampler.srcDY()) == 0) {
                            if (i != 0) {
                                memset(origTemp, 0, rowStart);
                                origTemp += rowStart;

                                for (int j = 0; j < width; j++) {
                                    memcpy(origTemp, bmRow, srcBytesPerPixel);
                                    origTemp += srcBytesPerPixel;
                                    bmRow += srcBytesPerPixel;
                                    if (j < (width -1)) {
                                        memset(origTemp, 0, (sampler.srcDX() - 1) * srcBytesPerPixel);
                                        origTemp += (sampler.srcDX() - 1) * srcBytesPerPixel;
                                    }
                                }

                                bmRow = row;
                                origTemp = origRow;
                            }

                            png_read_rows(png_ptr, &origTemp, png_bytepp_NULL, 1);
                            origTemp = origRow;

                            origTemp += rowStart;

                            for (int x = 0; x < width; x++) {
                                memcpy(bmRow, origTemp, srcBytesPerPixel);
                                bmRow +=srcBytesPerPixel;
                                origTemp += deltaSrc;
                            }

                            origTemp=origRow;
                            row += rb;
                        } else {
                            png_read_rows(png_ptr, &origTemp, png_bytepp_NULL, 1);
                            origTemp=origRow;
                        }
                    }
                }
                free(origRow);

                // now sample it
                for (int y = 0; y < height; y++) {
                    reallyHasAlpha |= sampler.next_without_step(base);
                    base += rb;
                }
            }
#else
            SkAutoMalloc storage(origWidth * origHeight * srcBytesPerPixel);
            uint8_t* base = (uint8_t*)storage.get();
            size_t rowBytes = origWidth * srcBytesPerPixel;

            for (int i = 0; i < number_passes; i++) {
                uint8_t* row = base;
                for (png_uint_32 y = 0; y < origHeight; y++) {
                    uint8_t* bmRow = row;
                    png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
                    row += rowBytes;
                }
            }
            // now sample it
            base += sampler.srcY0() * rowBytes;
            for (int y = 0; y < height; y++) {
                reallyHasAlpha |= sampler.next(base);
                base += sampler.srcDY() * rowBytes;
            }
#endif
            // MStar Android Patch End
        } else {
            SkAutoMalloc storage(origWidth * srcBytesPerPixel);
            uint8_t* srcRow = (uint8_t*)storage.get();
            skip_src_rows(png_ptr, srcRow, sampler.srcY0());

            for (int y = 0; y < height; y++) {
                uint8_t* tmp = srcRow;
                png_read_rows(png_ptr, &tmp, png_bytepp_NULL, 1);
                reallyHasAlpha |= sampler.next(srcRow);
                if (y < height - 1) {
                    skip_src_rows(png_ptr, srcRow, sampler.srcDY() - 1);
                }
            }

            // skip the rest of the rows (if any)
            png_uint_32 read = (height - 1) * sampler.srcDY() +
                               sampler.srcY0() + 1;
            SkASSERT(read <= origHeight);
            skip_src_rows(png_ptr, srcRow, origHeight - read);
        }
    }

    /* read rest of file, and get additional chunks in info_ptr - REQUIRED */
    png_read_end(png_ptr, info_ptr);

    if (0 != theTranspColor) {
        reallyHasAlpha |= substituteTranspColor(decodedBitmap, theTranspColor);
    }
    if (reallyHasAlpha && this->getRequireUnpremultipliedColors()) {
        switch (decodedBitmap->config()) {
            case SkBitmap::kIndex8_Config:
                // Fall through.
            case SkBitmap::kARGB_4444_Config:
                // We have chosen not to support unpremul for these configs.
                return false;
            default: {
                // Fall through to finish the decode. This config either
                // supports unpremul or it is irrelevant because it has no
                // alpha (or only alpha).
                // These brackets prevent a warning.
            }
        }
    }

    if (!reallyHasAlpha) {
        decodedBitmap->setAlphaType(kOpaque_SkAlphaType);
    }

    // MStar Android Patch Begin
#ifdef TEST_TIME_USED
    gettimeofday (&tv2 , 0);
    if (origWidth *origHeight >= 256*256)
        ALOGD("~!~!~ orgwidth=%d,orgHeight = %d, SW png decode end at:%ld ......\n",origWidth, origHeight,((tv2.tv_sec-tv.tv_sec)*1000+(tv2.tv_usec-tv.tv_usec)/1000));
#endif
    // MStar Android Patch End

    return true;
}



bool SkPNGImageDecoder::getBitmapConfig(png_structp png_ptr, png_infop info_ptr,
                                        SkBitmap::Config* SK_RESTRICT configp,
                                        bool* SK_RESTRICT hasAlphap,
                                        SkPMColor* SK_RESTRICT theTranspColorp) {
    png_uint_32 origWidth, origHeight;
    int bitDepth, colorType;
    png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bitDepth,
                 &colorType, int_p_NULL, int_p_NULL, int_p_NULL);

    // check for sBIT chunk data, in case we should disable dithering because
    // our data is not truely 8bits per component
    png_color_8p sig_bit;
    if (this->getDitherImage() && png_get_sBIT(png_ptr, info_ptr, &sig_bit)) {
#if 0
        SkDebugf("----- sBIT %d %d %d %d\n", sig_bit->red, sig_bit->green,
                 sig_bit->blue, sig_bit->alpha);
#endif
        // 0 seems to indicate no information available
        if (pos_le(sig_bit->red, SK_R16_BITS) &&
            pos_le(sig_bit->green, SK_G16_BITS) &&
            pos_le(sig_bit->blue, SK_B16_BITS)) {
            this->setDitherImage(false);
        }
    }

    if (colorType == PNG_COLOR_TYPE_PALETTE) {
        bool paletteHasAlpha = hasTransparencyInPalette(png_ptr, info_ptr);
        *configp = this->getPrefConfig(kIndex_SrcDepth, paletteHasAlpha);
        // now see if we can upscale to their requested config
        if (!canUpscalePaletteToConfig(*configp, paletteHasAlpha)) {
            *configp = SkBitmap::kIndex8_Config;
        }
    } else {
        png_color_16p transpColor = NULL;
        int numTransp = 0;

        png_get_tRNS(png_ptr, info_ptr, NULL, &numTransp, &transpColor);

        bool valid = png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS);

        if (valid && numTransp == 1 && transpColor != NULL) {
            /*  Compute our transparent color, which we'll match against later.
                We don't really handle 16bit components properly here, since we
                do our compare *after* the values have been knocked down to 8bit
                which means we will find more matches than we should. The real
                fix seems to be to see the actual 16bit components, do the
                compare, and then knock it down to 8bits ourselves.
            */
            if (colorType & PNG_COLOR_MASK_COLOR) {
                if (16 == bitDepth) {
                    *theTranspColorp = SkPackARGB32(0xFF, transpColor->red >> 8,
                                                    transpColor->green >> 8,
                                                    transpColor->blue >> 8);
                } else {
                    /* We apply the mask because in a very small
                       number of corrupt PNGs, (transpColor->red > 255)
                       and (bitDepth == 8), for certain versions of libpng. */
                    *theTranspColorp = SkPackARGB32(0xFF,
                                                    0xFF & (transpColor->red),
                                                    0xFF & (transpColor->green),
                                                    0xFF & (transpColor->blue));
                }
            } else {    // gray
                if (16 == bitDepth) {
                    *theTranspColorp = SkPackARGB32(0xFF, transpColor->gray >> 8,
                                                    transpColor->gray >> 8,
                                                    transpColor->gray >> 8);
                } else {
                    /* We apply the mask because in a very small
                       number of corrupt PNGs, (transpColor->red >
                       255) and (bitDepth == 8), for certain versions
                       of libpng.  For safety we assume the same could
                       happen with a grayscale PNG.  */
                    *theTranspColorp = SkPackARGB32(0xFF,
                                                    0xFF & (transpColor->gray),
                                                    0xFF & (transpColor->gray),
                                                    0xFF & (transpColor->gray));
                }
            }
        }

        if (valid ||
            PNG_COLOR_TYPE_RGB_ALPHA == colorType ||
            PNG_COLOR_TYPE_GRAY_ALPHA == colorType) {
            *hasAlphap = true;
        }

        SrcDepth srcDepth = k32Bit_SrcDepth;
        if (PNG_COLOR_TYPE_GRAY == colorType) {
            srcDepth = k8BitGray_SrcDepth;
            // Remove this assert, which fails on desk_pokemonwiki.skp
            //SkASSERT(!*hasAlphap);
        }

        *configp = this->getPrefConfig(srcDepth, *hasAlphap);
        // now match the request against our capabilities
        if (*hasAlphap) {
            if (*configp != SkBitmap::kARGB_4444_Config) {
                *configp = SkBitmap::kARGB_8888_Config;
            }
        } else {
            if (SkBitmap::kA8_Config == *configp) {
                if (k8BitGray_SrcDepth != srcDepth) {
                    // Converting a non grayscale image to A8 is not currently supported.
                    *configp = SkBitmap::kARGB_8888_Config;
                }
            } else if (*configp != SkBitmap::kRGB_565_Config &&
                       *configp != SkBitmap::kARGB_4444_Config) {
                *configp = SkBitmap::kARGB_8888_Config;
            }
        }
    }

    // sanity check for size
    {
        Sk64 size;
        size.setMul(origWidth, origHeight);
        if (size.isNeg() || !size.is32()) {
            return false;
        }
        // now check that if we are 4-bytes per pixel, we also don't overflow
        if (size.get32() > (0x7FFFFFFF >> 2)) {
            return false;
        }
    }

    if (!this->chooseFromOneChoice(*configp, origWidth, origHeight)) {
        return false;
    }

    // If the image has alpha and the decoder wants unpremultiplied
    // colors, the only supported config is 8888.
    if (this->getRequireUnpremultipliedColors() && *hasAlphap) {
        *configp = SkBitmap::kARGB_8888_Config;
    }

    if (fImageIndex != NULL) {
        if (SkBitmap::kNo_Config == fImageIndex->fConfig) {
            // This is the first time for this subset decode. From now on,
            // all decodes must be in the same config.
            fImageIndex->fConfig = *configp;
        } else if (fImageIndex->fConfig != *configp) {
            // Requesting a different config for a subsequent decode is not
            // supported. Report failure before we make changes to png_ptr.
            return false;
        }
    }

    bool convertGrayToRGB = PNG_COLOR_TYPE_GRAY == colorType
                            && *configp != SkBitmap::kA8_Config;

    // Unless the user is requesting A8, convert a grayscale image into RGB.
    // GRAY_ALPHA will always be converted to RGB
    if (convertGrayToRGB || colorType == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
    }

    // Add filler (or alpha) byte (after each RGB triplet) if necessary.
    if (colorType == PNG_COLOR_TYPE_RGB || convertGrayToRGB) {
        png_set_filler(png_ptr, 0xff, PNG_FILLER_AFTER);
    }

    return true;
}

typedef uint32_t (*PackColorProc)(U8CPU a, U8CPU r, U8CPU g, U8CPU b);

bool SkPNGImageDecoder::decodePalette(png_structp png_ptr, png_infop info_ptr,
                                      bool *hasAlphap, bool *reallyHasAlphap,
                                      SkColorTable **colorTablep) {
    int numPalette;
    png_colorp palette;
    png_bytep trans;
    int numTrans;

    png_get_PLTE(png_ptr, info_ptr, &palette, &numPalette);

    /*  BUGGY IMAGE WORKAROUND

        We hit some images (e.g. fruit_.png) who contain bytes that are == colortable_count
        which is a problem since we use the byte as an index. To work around this we grow
        the colortable by 1 (if its < 256) and duplicate the last color into that slot.
    */
    int colorCount = numPalette + (numPalette < 256);
    SkPMColor colorStorage[256];    // worst-case storage
    SkPMColor* colorPtr = colorStorage;

    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
        png_get_tRNS(png_ptr, info_ptr, &trans, &numTrans, NULL);
        *hasAlphap = (numTrans > 0);
    } else {
        numTrans = 0;
    }

    // check for bad images that might make us crash
    if (numTrans > numPalette) {
        numTrans = numPalette;
    }

    int index = 0;
    int transLessThanFF = 0;

    // Choose which function to use to create the color table. If the final destination's
    // config is unpremultiplied, the color table will store unpremultiplied colors.
    PackColorProc proc;
    if (this->getRequireUnpremultipliedColors()) {
        proc = &SkPackARGB32NoCheck;
    } else {
        proc = &SkPreMultiplyARGB;
    }
    for (; index < numTrans; index++) {
        transLessThanFF |= (int)*trans - 0xFF;
        *colorPtr++ = proc(*trans++, palette->red, palette->green, palette->blue);
        palette++;
    }
    bool reallyHasAlpha = (transLessThanFF < 0);

    for (; index < numPalette; index++) {
        *colorPtr++ = SkPackARGB32(0xFF, palette->red, palette->green, palette->blue);
        palette++;
    }

    // see BUGGY IMAGE WORKAROUND comment above
    if (numPalette < 256) {
        *colorPtr = colorPtr[-1];
    }

    SkAlphaType alphaType = kOpaque_SkAlphaType;
    if (reallyHasAlpha) {
        if (this->getRequireUnpremultipliedColors()) {
            alphaType = kUnpremul_SkAlphaType;
        } else {
            alphaType = kPremul_SkAlphaType;
        }
    }

    *colorTablep = SkNEW_ARGS(SkColorTable,
                              (colorStorage, colorCount, alphaType));
    *reallyHasAlphap = reallyHasAlpha;
    return true;
}

#ifdef SK_BUILD_FOR_ANDROID

bool SkPNGImageDecoder::onBuildTileIndex(SkStreamRewindable* sk_stream, int *width, int *height) {
    png_structp png_ptr;
    png_infop   info_ptr;

    if (!onDecodeInit(sk_stream, &png_ptr, &info_ptr)) {
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr)) != 0) {
        png_destroy_read_struct(&png_ptr, &info_ptr, png_infopp_NULL);
        return false;
    }

    png_uint_32 origWidth, origHeight;
    int bitDepth, colorType;
    png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bitDepth,
                 &colorType, int_p_NULL, int_p_NULL, int_p_NULL);

    *width = origWidth;
    *height = origHeight;

    png_build_index(png_ptr);

    if (fImageIndex) {
        SkDELETE(fImageIndex);
    }
    fImageIndex = SkNEW_ARGS(SkPNGImageIndex, (sk_stream, png_ptr, info_ptr));

    return true;
}

bool SkPNGImageDecoder::onDecodeSubset(SkBitmap* bm, const SkIRect& region) {
    if (NULL == fImageIndex) {
        return false;
    }

    png_structp png_ptr = fImageIndex->fPng_ptr;
    png_infop info_ptr = fImageIndex->fInfo_ptr;
    if (setjmp(png_jmpbuf(png_ptr))) {
        return false;
    }

    png_uint_32 origWidth, origHeight;
    int bitDepth, colorType, interlaceType;
    png_get_IHDR(png_ptr, info_ptr, &origWidth, &origHeight, &bitDepth,
                 &colorType, &interlaceType, int_p_NULL, int_p_NULL);

    SkIRect rect = SkIRect::MakeWH(origWidth, origHeight);

    if (!rect.intersect(region)) {
        // If the requested region is entirely outside the image, just
        // returns false
        return false;
    }

    SkBitmap::Config    config;
    bool                hasAlpha = false;
    SkPMColor           theTranspColor = 0; // 0 tells us not to try to match

    if (!this->getBitmapConfig(png_ptr, info_ptr, &config, &hasAlpha, &theTranspColor)) {
        return false;
    }

    const int sampleSize = this->getSampleSize();
    SkScaledBitmapSampler sampler(origWidth, rect.height(), sampleSize);

    SkBitmap decodedBitmap;
    decodedBitmap.setConfig(config, sampler.scaledWidth(), sampler.scaledHeight());

    // from here down we are concerned with colortables and pixels

    // we track if we actually see a non-opaque pixels, since sometimes a PNG sets its colortype
    // to |= PNG_COLOR_MASK_ALPHA, but all of its pixels are in fact opaque. We care, since we
    // draw lots faster if we can flag the bitmap has being opaque
    bool reallyHasAlpha = false;
    SkColorTable* colorTable = NULL;

    if (colorType == PNG_COLOR_TYPE_PALETTE) {
        decodePalette(png_ptr, info_ptr, &hasAlpha, &reallyHasAlpha, &colorTable);
    }

    SkAutoUnref aur(colorTable);

    // Check ahead of time if the swap(dest, src) is possible.
    // If yes, then we will stick to AllocPixelRef since it's cheaper with the swap happening.
    // If no, then we will use alloc to allocate pixels to prevent garbage collection.
    int w = rect.width() / sampleSize;
    int h = rect.height() / sampleSize;
    const bool swapOnly = (rect == region) && (w == decodedBitmap.width()) &&
                          (h == decodedBitmap.height()) && bm->isNull();
    const bool needColorTable = SkBitmap::kIndex8_Config == config;
    if (swapOnly) {
        if (!this->allocPixelRef(&decodedBitmap, needColorTable ? colorTable : NULL)) {
            return false;
        }
    } else {
        if (!decodedBitmap.allocPixels(NULL, needColorTable ? colorTable : NULL)) {
            return false;
        }
    }
    SkAutoLockPixels alp(decodedBitmap);

    /* Turn on interlace handling.  REQUIRED if you are not using
    * png_read_image().  To see how to handle interlacing passes,
    * see the png_read_row() method below:
    */
    const int number_passes = (interlaceType != PNG_INTERLACE_NONE) ?
                              png_set_interlace_handling(png_ptr) : 1;

    /* Optional call to gamma correct and add the background to the palette
    * and update info structure.  REQUIRED if you are expecting libpng to
    * update the palette for you (ie you selected such a transform above).
    */

    // Direct access to png_ptr fields is deprecated in libpng > 1.2.
#if defined(PNG_1_0_X) || defined (PNG_1_2_X)
    png_ptr->pass = 0;
#else
    // FIXME: This sets pass as desired, but also sets iwidth. Is that ok?
    png_set_interlaced_pass(png_ptr, 0);
#endif
    png_read_update_info(png_ptr, info_ptr);

    int actualTop = rect.fTop;

    if ((SkBitmap::kA8_Config == config || SkBitmap::kIndex8_Config == config)
        && 1 == sampleSize) {
        if (SkBitmap::kA8_Config == config) {
            // For an A8 bitmap, we assume there is an alpha for speed. It is
            // possible the bitmap is opaque, but that is an unlikely use case
            // since it would not be very interesting.
            reallyHasAlpha = true;
            // A8 is only allowed if the original was GRAY.
            SkASSERT(PNG_COLOR_TYPE_GRAY == colorType);
        }

        for (int i = 0; i < number_passes; i++) {
            png_configure_decoder(png_ptr, &actualTop, i);
            for (int j = 0; j < rect.fTop - actualTop; j++) {
                uint8_t* bmRow = decodedBitmap.getAddr8(0, 0);
                png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
            }
            png_uint_32 bitmapHeight = (png_uint_32) decodedBitmap.height();
            for (png_uint_32 y = 0; y < bitmapHeight; y++) {
                uint8_t* bmRow = decodedBitmap.getAddr8(0, y);
                png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
            }
        }
    } else {
        SkScaledBitmapSampler::SrcConfig sc;
        int srcBytesPerPixel = 4;

        if (colorTable != NULL) {
            sc = SkScaledBitmapSampler::kIndex;
            srcBytesPerPixel = 1;
        } else if (SkBitmap::kA8_Config == config) {
            // A8 is only allowed if the original was GRAY.
            SkASSERT(PNG_COLOR_TYPE_GRAY == colorType);
            sc = SkScaledBitmapSampler::kGray;
            srcBytesPerPixel = 1;
        } else if (hasAlpha) {
            sc = SkScaledBitmapSampler::kRGBA;
        } else {
            sc = SkScaledBitmapSampler::kRGBX;
        }

        /*  We have to pass the colortable explicitly, since we may have one
            even if our decodedBitmap doesn't, due to the request that we
            upscale png's palette to a direct model
         */
        SkAutoLockColors ctLock(colorTable);
        if (!sampler.begin(&decodedBitmap, sc, *this, ctLock.colors())) {
            return false;
        }
        const int height = decodedBitmap.height();

        if (number_passes > 1) {
            SkAutoMalloc storage(origWidth * origHeight * srcBytesPerPixel);
            uint8_t* base = (uint8_t*)storage.get();
            size_t rb = origWidth * srcBytesPerPixel;

            for (int i = 0; i < number_passes; i++) {
                png_configure_decoder(png_ptr, &actualTop, i);
                for (int j = 0; j < rect.fTop - actualTop; j++) {
                    uint8_t* bmRow = (uint8_t*)decodedBitmap.getPixels();
                    png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
                }
                uint8_t* row = base;
                // MStar Android Patch Begin
                for (int32_t y = 0; y < (png_uint_32)rect.height(); y++) {
                // MStar Android Patch End
                    uint8_t* bmRow = row;
                    png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
                    row += rb;
                }
            }
            // now sample it
            base += sampler.srcY0() * rb;
            for (int y = 0; y < height; y++) {
                reallyHasAlpha |= sampler.next(base);
                base += sampler.srcDY() * rb;
            }
        } else {
            SkAutoMalloc storage(origWidth * srcBytesPerPixel);
            uint8_t* srcRow = (uint8_t*)storage.get();

            png_configure_decoder(png_ptr, &actualTop, 0);
            skip_src_rows(png_ptr, srcRow, sampler.srcY0());

            for (int i = 0; i < rect.fTop - actualTop; i++) {
                uint8_t* bmRow = (uint8_t*)decodedBitmap.getPixels();
                png_read_rows(png_ptr, &bmRow, png_bytepp_NULL, 1);
            }
            for (int y = 0; y < height; y++) {
                uint8_t* tmp = srcRow;
                png_read_rows(png_ptr, &tmp, png_bytepp_NULL, 1);
                reallyHasAlpha |= sampler.next(srcRow);
                if (y < height - 1) {
                    skip_src_rows(png_ptr, srcRow, sampler.srcDY() - 1);
                }
            }
        }
    }

    if (0 != theTranspColor) {
        reallyHasAlpha |= substituteTranspColor(&decodedBitmap, theTranspColor);
    }
    if (reallyHasAlpha && this->getRequireUnpremultipliedColors()) {
        switch (decodedBitmap.config()) {
            case SkBitmap::kIndex8_Config:
                // Fall through.
            case SkBitmap::kARGB_4444_Config:
                // We have chosen not to support unpremul for these configs.
                return false;
            default: {
                // Fall through to finish the decode. This config either
                // supports unpremul or it is irrelevant because it has no
                // alpha (or only alpha).
                // These brackets prevent a warning.
            }
        }
    }
    SkAlphaType alphaType = kOpaque_SkAlphaType;
    if (reallyHasAlpha) {
        if (this->getRequireUnpremultipliedColors()) {
            alphaType = kUnpremul_SkAlphaType;
        } else {
            alphaType = kPremul_SkAlphaType;
        }
    }
    decodedBitmap.setAlphaType(alphaType);

    if (swapOnly) {
        bm->swap(decodedBitmap);
        return true;
    }
    return this->cropBitmap(bm, &decodedBitmap, sampleSize, region.x(), region.y(),
                            region.width(), region.height(), 0, rect.y());
}
#endif

///////////////////////////////////////////////////////////////////////////////

#include "SkColorPriv.h"
#include "SkUnPreMultiply.h"

static void sk_write_fn(png_structp png_ptr, png_bytep data, png_size_t len) {
    SkWStream* sk_stream = (SkWStream*)png_get_io_ptr(png_ptr);
    if (!sk_stream->write(data, len)) {
        png_error(png_ptr, "sk_write_fn Error!");
    }
}

static transform_scanline_proc choose_proc(SkBitmap::Config config,
                                           bool hasAlpha) {
    // we don't care about search on alpha if we're kIndex8, since only the
    // colortable packing cares about that distinction, not the pixels
    if (SkBitmap::kIndex8_Config == config) {
        hasAlpha = false;   // we store false in the table entries for kIndex8
    }

    static const struct {
        SkBitmap::Config        fConfig;
        bool                    fHasAlpha;
        transform_scanline_proc fProc;
    } gMap[] = {
        { SkBitmap::kRGB_565_Config,    false,  transform_scanline_565 },
        { SkBitmap::kARGB_8888_Config,  false,  transform_scanline_888 },
        { SkBitmap::kARGB_8888_Config,  true,   transform_scanline_8888 },
        { SkBitmap::kARGB_4444_Config,  false,  transform_scanline_444 },
        { SkBitmap::kARGB_4444_Config,  true,   transform_scanline_4444 },
        { SkBitmap::kIndex8_Config,     false,  transform_scanline_memcpy },
    };

    for (int i = SK_ARRAY_COUNT(gMap) - 1; i >= 0; --i) {
        if (gMap[i].fConfig == config && gMap[i].fHasAlpha == hasAlpha) {
            return gMap[i].fProc;
        }
    }
    sk_throw();
    return NULL;
}

// return the minimum legal bitdepth (by png standards) for this many colortable
// entries. SkBitmap always stores in 8bits per pixel, but for colorcount <= 16,
// we can use fewer bits per in png
static int computeBitDepth(int colorCount) {
#if 0
    int bits = SkNextLog2(colorCount);
    SkASSERT(bits >= 1 && bits <= 8);
    // now we need bits itself to be a power of 2 (e.g. 1, 2, 4, 8)
    return SkNextPow2(bits);
#else
    // for the moment, we don't know how to pack bitdepth < 8
    return 8;
#endif
}

/*  Pack palette[] with the corresponding colors, and if hasAlpha is true, also
    pack trans[] and return the number of trans[] entries written. If hasAlpha
    is false, the return value will always be 0.

    Note: this routine takes care of unpremultiplying the RGB values when we
    have alpha in the colortable, since png doesn't support premul colors
*/
static inline int pack_palette(SkColorTable* ctable,
                               png_color* SK_RESTRICT palette,
                               png_byte* SK_RESTRICT trans, bool hasAlpha) {
    SkAutoLockColors alc(ctable);
    const SkPMColor* SK_RESTRICT colors = alc.colors();
    const int ctCount = ctable->count();
    int i, num_trans = 0;

    if (hasAlpha) {
        /*  first see if we have some number of fully opaque at the end of the
            ctable. PNG allows num_trans < num_palette, but all of the trans
            entries must come first in the palette. If I was smarter, I'd
            reorder the indices and ctable so that all non-opaque colors came
            first in the palette. But, since that would slow down the encode,
            I'm leaving the indices and ctable order as is, and just looking
            at the tail of the ctable for opaqueness.
        */
        num_trans = ctCount;
        for (i = ctCount - 1; i >= 0; --i) {
            if (SkGetPackedA32(colors[i]) != 0xFF) {
                break;
            }
            num_trans -= 1;
        }

        const SkUnPreMultiply::Scale* SK_RESTRICT table =
                                            SkUnPreMultiply::GetScaleTable();

        for (i = 0; i < num_trans; i++) {
            const SkPMColor c = *colors++;
            const unsigned a = SkGetPackedA32(c);
            const SkUnPreMultiply::Scale s = table[a];
            trans[i] = a;
            palette[i].red = SkUnPreMultiply::ApplyScale(s, SkGetPackedR32(c));
            palette[i].green = SkUnPreMultiply::ApplyScale(s,SkGetPackedG32(c));
            palette[i].blue = SkUnPreMultiply::ApplyScale(s, SkGetPackedB32(c));
        }
        // now fall out of this if-block to use common code for the trailing
        // opaque entries
    }

    // these (remaining) entries are opaque
    for (i = num_trans; i < ctCount; i++) {
        SkPMColor c = *colors++;
        palette[i].red = SkGetPackedR32(c);
        palette[i].green = SkGetPackedG32(c);
        palette[i].blue = SkGetPackedB32(c);
    }
    return num_trans;
}

class SkPNGImageEncoder : public SkImageEncoder {
protected:
    virtual bool onEncode(SkWStream* stream, const SkBitmap& bm, int quality) SK_OVERRIDE;
private:
    bool doEncode(SkWStream* stream, const SkBitmap& bm,
                  const bool& hasAlpha, int colorType,
                  int bitDepth, SkBitmap::Config config,
                  png_color_8& sig_bit);

    typedef SkImageEncoder INHERITED;
};

bool SkPNGImageEncoder::onEncode(SkWStream* stream, const SkBitmap& bitmap,
                                 int /*quality*/) {
    SkBitmap::Config config = bitmap.config();

    const bool hasAlpha = !bitmap.isOpaque();
    int colorType = PNG_COLOR_MASK_COLOR;
    int bitDepth = 8;   // default for color
    png_color_8 sig_bit;

    switch (config) {
        case SkBitmap::kIndex8_Config:
            colorType |= PNG_COLOR_MASK_PALETTE;
            // fall through to the ARGB_8888 case
        case SkBitmap::kARGB_8888_Config:
            sig_bit.red = 8;
            sig_bit.green = 8;
            sig_bit.blue = 8;
            sig_bit.alpha = 8;
            break;
        case SkBitmap::kARGB_4444_Config:
            sig_bit.red = 4;
            sig_bit.green = 4;
            sig_bit.blue = 4;
            sig_bit.alpha = 4;
            break;
        case SkBitmap::kRGB_565_Config:
            sig_bit.red = 5;
            sig_bit.green = 6;
            sig_bit.blue = 5;
            sig_bit.alpha = 0;
            break;
        default:
            return false;
    }

    if (hasAlpha) {
        // don't specify alpha if we're a palette, even if our ctable has alpha
        if (!(colorType & PNG_COLOR_MASK_PALETTE)) {
            colorType |= PNG_COLOR_MASK_ALPHA;
        }
    } else {
        sig_bit.alpha = 0;
    }

    SkAutoLockPixels alp(bitmap);
    // readyToDraw checks for pixels (and colortable if that is required)
    if (!bitmap.readyToDraw()) {
        return false;
    }

    // we must do this after we have locked the pixels
    SkColorTable* ctable = bitmap.getColorTable();
    if (NULL != ctable) {
        if (ctable->count() == 0) {
            return false;
        }
        // check if we can store in fewer than 8 bits
        bitDepth = computeBitDepth(ctable->count());
    }

    return doEncode(stream, bitmap, hasAlpha, colorType,
                    bitDepth, config, sig_bit);
}

bool SkPNGImageEncoder::doEncode(SkWStream* stream, const SkBitmap& bitmap,
                  const bool& hasAlpha, int colorType,
                  int bitDepth, SkBitmap::Config config,
                  png_color_8& sig_bit) {

    png_structp png_ptr;
    png_infop info_ptr;

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, sk_error_fn,
                                      NULL);
    if (NULL == png_ptr) {
        return false;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (NULL == info_ptr) {
        png_destroy_write_struct(&png_ptr,  png_infopp_NULL);
        return false;
    }

    /* Set error handling.  REQUIRED if you aren't supplying your own
    * error handling functions in the png_create_write_struct() call.
    */
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    png_set_write_fn(png_ptr, (void*)stream, sk_write_fn, png_flush_ptr_NULL);

    /* Set the image information here.  Width and height are up to 2^31,
    * bit_depth is one of 1, 2, 4, 8, or 16, but valid values also depend on
    * the color_type selected. color_type is one of PNG_COLOR_TYPE_GRAY,
    * PNG_COLOR_TYPE_GRAY_ALPHA, PNG_COLOR_TYPE_PALETTE, PNG_COLOR_TYPE_RGB,
    * or PNG_COLOR_TYPE_RGB_ALPHA.  interlace is either PNG_INTERLACE_NONE or
    * PNG_INTERLACE_ADAM7, and the compression_type and filter_type MUST
    * currently be PNG_COMPRESSION_TYPE_BASE and PNG_FILTER_TYPE_BASE. REQUIRED
    */

    png_set_IHDR(png_ptr, info_ptr, bitmap.width(), bitmap.height(),
                 bitDepth, colorType,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
                 PNG_FILTER_TYPE_BASE);

    // set our colortable/trans arrays if needed
    png_color paletteColors[256];
    png_byte trans[256];
    if (SkBitmap::kIndex8_Config == config) {
        SkColorTable* ct = bitmap.getColorTable();
        int numTrans = pack_palette(ct, paletteColors, trans, hasAlpha);
        png_set_PLTE(png_ptr, info_ptr, paletteColors, ct->count());
        if (numTrans > 0) {
            png_set_tRNS(png_ptr, info_ptr, trans, numTrans, NULL);
        }
    }

    png_set_sBIT(png_ptr, info_ptr, &sig_bit);
    png_write_info(png_ptr, info_ptr);

    const char* srcImage = (const char*)bitmap.getPixels();
    SkAutoSMalloc<1024> rowStorage(bitmap.width() << 2);
    char* storage = (char*)rowStorage.get();
    transform_scanline_proc proc = choose_proc(config, hasAlpha);

    for (int y = 0; y < bitmap.height(); y++) {
        png_bytep row_ptr = (png_bytep)storage;
        proc(srcImage, bitmap.width(), storage);
        png_write_rows(png_ptr, &row_ptr, 1);
        srcImage += bitmap.rowBytes();
    }

    png_write_end(png_ptr, info_ptr);

    /* clean up after the write, and free any memory allocated */
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return true;
}

///////////////////////////////////////////////////////////////////////////////
DEFINE_DECODER_CREATOR(PNGImageDecoder);
DEFINE_ENCODER_CREATOR(PNGImageEncoder);
///////////////////////////////////////////////////////////////////////////////

static bool is_png(SkStreamRewindable* stream) {
    char buf[PNG_BYTES_TO_CHECK];
    if (stream->read(buf, PNG_BYTES_TO_CHECK) == PNG_BYTES_TO_CHECK &&
        !png_sig_cmp((png_bytep) buf, (png_size_t)0, PNG_BYTES_TO_CHECK)) {
        return true;
    }
    return false;
}

SkImageDecoder* sk_libpng_dfactory(SkStreamRewindable* stream) {
    if (is_png(stream)) {
        return SkNEW(SkPNGImageDecoder);
    }
    return NULL;
}

static SkImageDecoder::Format get_format_png(SkStreamRewindable* stream) {
    if (is_png(stream)) {
        return SkImageDecoder::kPNG_Format;
    }
    return SkImageDecoder::kUnknown_Format;
}

SkImageEncoder* sk_libpng_efactory(SkImageEncoder::Type t) {
    return (SkImageEncoder::kPNG_Type == t) ? SkNEW(SkPNGImageEncoder) : NULL;
}

static SkImageDecoder_DecodeReg gDReg(sk_libpng_dfactory);
static SkImageDecoder_FormatReg gFormatReg(get_format_png);
static SkImageEncoder_EncodeReg gEReg(sk_libpng_efactory);
