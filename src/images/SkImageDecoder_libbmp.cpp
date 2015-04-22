
/*
 * Copyright 2007 The Android Open Source Project
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */


#include "bmpdecoderhelper.h"
#include "SkColorPriv.h"
#include "SkImageDecoder.h"
#include "SkScaledBitmapSampler.h"
#include "SkStream.h"
#include "SkStreamHelpers.h"
#include "SkTDArray.h"

// MStar Android Patch Begin
#include <unistd.h>
#include <utils/Log.h>

#define READ_BMP_HEADER_LENGTH_MAX   2048
#define READ_BMP_DATA_MAX_LENGTH_EVERY_TIME   (256 * 1024)
#define ENABLE_BMP_DECODE_OPTIMIZE   1
#define BMP_MAX_WIDTH    (1920 * 8)
#define BMP_MAX_HEIGHT    (1080 * 8)
// MStar Android Patch End

class SkBMPImageDecoder : public SkImageDecoder {
public:
    SkBMPImageDecoder() {}

    virtual Format getFormat() const SK_OVERRIDE {
        return kBMP_Format;
    }

protected:
    virtual bool onDecode(SkStream* stream, SkBitmap* bm, Mode mode) SK_OVERRIDE;

private:
    typedef SkImageDecoder INHERITED;
};

///////////////////////////////////////////////////////////////////////////////
DEFINE_DECODER_CREATOR(BMPImageDecoder);
///////////////////////////////////////////////////////////////////////////////

static bool is_bmp(SkStreamRewindable* stream) {
    static const char kBmpMagic[] = { 'B', 'M' };


    char buffer[sizeof(kBmpMagic)];

    return stream->read(buffer, sizeof(kBmpMagic)) == sizeof(kBmpMagic) &&
        !memcmp(buffer, kBmpMagic, sizeof(kBmpMagic));
}

static SkImageDecoder* sk_libbmp_dfactory(SkStreamRewindable* stream) {
    if (is_bmp(stream)) {
        return SkNEW(SkBMPImageDecoder);
    }
    return NULL;
}

static SkImageDecoder_DecodeReg gReg(sk_libbmp_dfactory);

static SkImageDecoder::Format get_format_bmp(SkStreamRewindable* stream) {
    if (is_bmp(stream)) {
        return SkImageDecoder::kBMP_Format;
    }
    return SkImageDecoder::kUnknown_Format;
}

static SkImageDecoder_FormatReg gFormatReg(get_format_bmp);

///////////////////////////////////////////////////////////////////////////////

class SkBmpDecoderCallback : public image_codec::BmpDecoderCallback {
public:
    // we don't copy the bitmap, just remember the pointer
    SkBmpDecoderCallback(bool justBounds) : fJustBounds(justBounds) {}

    // override from BmpDecoderCallback
    virtual uint8* SetSize(int width, int height) {
        fWidth = width;
        fHeight = height;
        if (fJustBounds) {
            return NULL;
        }

        fRGB.setCount(width * height * 3);  // 3 == r, g, b
        return fRGB.begin();
    }

    int width() const { return fWidth; }
    int height() const { return fHeight; }
    const uint8_t* rgb() const { return fRGB.begin(); }

private:
    SkTDArray<uint8_t> fRGB;
    int fWidth;
    int fHeight;
    bool fJustBounds;
};

bool SkBMPImageDecoder::onDecode(SkStream* stream, SkBitmap* bm, Mode mode) {
    // MStar Android Patch Begin
#if ENABLE_BMP_DECODE_OPTIMIZE
    SkASSERT(stream != NULL);

    SkAutoMalloc storage;
    size_t readSize = 0;
    size_t haveRead = 0;
    const bool justBounds = SkImageDecoder::kDecodeBounds_Mode == mode;

    if (justBounds)
        readSize = READ_BMP_HEADER_LENGTH_MAX;
    else
        readSize = READ_BMP_DATA_MAX_LENGTH_EVERY_TIME;

    storage.reset(readSize);
    haveRead = stream->read(storage.get(), readSize);
    if (haveRead < 1) {
        ALOGE("[BMP] error: first read error!");
        return false;
    }

    SkBmpDecoderCallback callback(justBounds);
    image_codec::BmpDecoderHelper helper;
    const int max_pixels = 16383*16383; // max width*height

    if (justBounds) {
        if (!helper.DecodeImage((const char*)storage.get(), haveRead, max_pixels, &callback)) {
            ALOGE("[BMP] error: decode resolution fail 1!");
            return false;
        }
    } else {
        // 1. get width and height
        SkBmpDecoderCallback callback_ForGetInfo(true);
        if (!helper.DecodeImage((const char*)storage.get(), haveRead, max_pixels, &callback_ForGetInfo)) {
            ALOGE("[BMP] error: decode resolution fail 2!");
            return false;
        }

        int width = callback_ForGetInfo.width();
        int height = callback_ForGetInfo.height();

        if ((width * height) > (BMP_MAX_WIDTH * BMP_MAX_HEIGHT)) {
            return false;
        }

        SkBitmap::Config config = this->getPrefConfig(k32Bit_SrcDepth, false);

        // only accept prefConfig if it makes sense for us
        if (SkBitmap::kARGB_4444_Config != config && SkBitmap::kRGB_565_Config != config) {
            config = SkBitmap::kARGB_8888_Config;
        }

        SkScaledBitmapSampler sampler(width, height, getSampleSize());
        bm->setConfig(config, sampler.scaledWidth(), sampler.scaledHeight());
        // @MSTARFIXME
        //bm->setIsOpaque(true);

        if (!this->allocPixelRef(bm, NULL)) {
            return false;
        }

        SkAutoLockPixels alp(*bm);

        if (!sampler.begin(bm, SkScaledBitmapSampler::kRGB, *this)) {
            return false;
        }

        // 2. call DecodeImageEx to decode and do scale down
        if (!helper.DecodeImageEx(stream, (const char*)storage.get(), haveRead, max_pixels, &callback, &sampler, this)) {
            ALOGE("[BMP] error: decode data fail!");
            return false;
        }

        // we don't need this anymore, so free it now (before we try to allocate
        // the bitmap's pixels) rather than waiting for its destructor
        storage.free();
        return true;
    }
#else
    // First read the entire stream, so that all of the data can be passed to
    // the BmpDecoderHelper.

    // Allocated space used to hold the data.
    SkAutoMalloc storage;
    // Byte length of all of the data.
    const size_t length = CopyStreamToStorage(&storage, stream);
    if (0 == length) {
        return 0;
    }

    const bool justBounds = SkImageDecoder::kDecodeBounds_Mode == mode;
    SkBmpDecoderCallback callback(justBounds);

    // Now decode the BMP into callback's rgb() array [r,g,b, r,g,b, ...]
    {
        image_codec::BmpDecoderHelper helper;
        const int max_pixels = 16383*16383; // max width*height
        if (!helper.DecodeImage((const char*)storage.get(), length,
                                max_pixels, &callback)) {
            return false;
        }
    }
#endif
    // MStar Android Patch End

    // we don't need this anymore, so free it now (before we try to allocate
    // the bitmap's pixels) rather than waiting for its destructor
    storage.free();

    int width = callback.width();
    int height = callback.height();
    SkBitmap::Config config = this->getPrefConfig(k32Bit_SrcDepth, false);

    // only accept prefConfig if it makes sense for us
    if (SkBitmap::kARGB_4444_Config != config &&
            SkBitmap::kRGB_565_Config != config) {
        config = SkBitmap::kARGB_8888_Config;
    }

    SkScaledBitmapSampler sampler(width, height, getSampleSize());

    bm->setConfig(config, sampler.scaledWidth(), sampler.scaledHeight(), 0,
                  kOpaque_SkAlphaType);

    if (justBounds) {
        return true;
    }

    if (!this->allocPixelRef(bm, NULL)) {
        return false;
    }

    SkAutoLockPixels alp(*bm);

    if (!sampler.begin(bm, SkScaledBitmapSampler::kRGB, *this)) {
        return false;
    }

    const int srcRowBytes = width * 3;
    const int dstHeight = sampler.scaledHeight();
    const uint8_t* srcRow = callback.rgb();

    srcRow += sampler.srcY0() * srcRowBytes;
    for (int y = 0; y < dstHeight; y++) {
        sampler.next(srcRow);
        srcRow += sampler.srcDY() * srcRowBytes;
    }
    return true;
}
