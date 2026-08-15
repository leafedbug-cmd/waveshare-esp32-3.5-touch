/**
 * @file lv_conf.h
 * LVGL v9.5 configuration for Waveshare ESP32-S3-Touch-LCD-3.5-C
 *
 * ST7796 320x480 IPS, landscape orientation (480x320).
 * 8MB OPI PSRAM used for LVGL heap, draw buffers, and image cache.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
   COLOR SETTINGS
 *====================*/

#define LV_COLOR_DEPTH 16  /* RGB565 for ST7796 */

/*=========================
   STDLIB WRAPPER SETTINGS
 *=========================*/

#define LV_USE_STDLIB_MALLOC    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING    LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_BUILTIN

#define LV_STDINT_INCLUDE       <stdint.h>
#define LV_STDDEF_INCLUDE       <stddef.h>
#define LV_STDBOOL_INCLUDE      <stdbool.h>
#define LV_INTTYPES_INCLUDE     <inttypes.h>
#define LV_LIMITS_INCLUDE       <limits.h>
#define LV_STDARG_INCLUDE       <stdarg.h>

/* LVGL builtin memory pool: 512KB in PSRAM */
#define LV_MEM_SIZE (512U * 1024U)
#define LV_MEM_POOL_EXPAND_SIZE 0
#define LV_MEM_ADR 0
#define LV_MEM_POOL_INCLUDE "esp32-hal-psram.h"
#define LV_MEM_POOL_ALLOC ps_malloc

/*====================
   HAL SETTINGS
 *====================*/

#define LV_DEF_REFR_PERIOD  33    /* ~30 fps */
#define LV_DPI_DEF          130

/*=================
 * OPERATING SYSTEM
 *=================*/

#define LV_USE_OS   LV_OS_NONE

/*========================
 * RENDERING CONFIGURATION
 *========================*/

#define LV_DRAW_BUF_STRIDE_ALIGN    1
#define LV_DRAW_BUF_ALIGN           4
#define LV_DRAW_TRANSFORM_USE_MATRIX 0
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE (24 * 1024)
#define LV_DRAW_LAYER_MAX_MEMORY 0
#define LV_DRAW_THREAD_STACK_SIZE (8 * 1024)

#define LV_USE_DRAW_SW 1
#if LV_USE_DRAW_SW == 1
    #define LV_DRAW_SW_SUPPORT_RGB565       1
    #define LV_DRAW_SW_SUPPORT_RGB565_SWAPPED 1
    #define LV_DRAW_SW_SUPPORT_RGB565A8     1
    #define LV_DRAW_SW_SUPPORT_RGB888       1
    #define LV_DRAW_SW_SUPPORT_XRGB8888    1
    #define LV_DRAW_SW_SUPPORT_ARGB8888    1
    #define LV_DRAW_SW_SUPPORT_ARGB8888_PREMULTIPLIED 1
    #define LV_DRAW_SW_SUPPORT_L8          1
    #define LV_DRAW_SW_SUPPORT_AL88        1
    #define LV_DRAW_SW_SUPPORT_A8          1
    #define LV_DRAW_SW_SUPPORT_I1          1
    #define LV_DRAW_SW_I1_LUM_THRESHOLD    127
    #define LV_DRAW_SW_DRAW_UNIT_CNT       1
    #define LV_USE_DRAW_ARM2D_SYNC         0
    #define LV_DRAW_SW_COMPLEX             1
    #if LV_DRAW_SW_COMPLEX == 1
        #define LV_DRAW_SW_SHADOW_CACHE_SIZE 0
        #define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4
    #endif
    #define LV_USE_DRAW_SW_ASM LV_DRAW_SW_ASM_NONE
    #define LV_USE_DRAW_SW_COMPLEX_GRADIENTS 0
#endif

/* Disable GPU renderers not applicable to ESP32 */
#define LV_USE_NEMA_GFX 0
#define LV_USE_PXP      0
#define LV_USE_G2D      0
#define LV_USE_DRAW_VG_LITE 0
#define LV_USE_DRAW_OPENGLES 0
#define LV_USE_DRAW_SDL 0
#define LV_USE_DRAW_NANOVG 0

/*=======================
 * FEATURE CONFIGURATION
 *=======================*/

/*-------------
 * Logging
 *-----------*/

#define LV_USE_LOG 1
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    #define LV_LOG_PRINTF 1
#endif

/*-------------
 * Asserts
 *-----------*/

#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/*-------------
 * Debug
 *-----------*/

#define LV_USE_REFR_DEBUG       0
#define LV_USE_LAYER_DEBUG      0
#define LV_USE_PARALLEL_DRAW_DEBUG 0

/*-------------
 * Others
 *-----------*/

#define LV_ENABLE_GLOBAL_CUSTOM 0
#define LV_USE_SYSMON 0

/* Image cache: decoded image data kept in PSRAM (512KB budget) */
#define LV_CACHE_DEF_SIZE          (512 * 1024)
#define LV_IMAGE_HEADER_CACHE_DEF_CNT 32

#define LV_GRADIENT_MAX_STOPS      2
#define LV_COLOR_MIX_ROUND_OFS     0
#define LV_OBJ_STYLE_CACHE         0
#define LV_USE_OBJ_ID              0
#define LV_USE_OBJ_NAME            0
#define LV_OBJ_ID_AUTO_ASSIGN      LV_USE_OBJ_ID
#define LV_USE_OBJ_ID_BUILTIN      1
#define LV_USE_OBJ_PROPERTY        0
#define LV_USE_OBJ_PROPERTY_NAME   1
#define LV_USE_GESTURE_RECOGNITION 0

/*=====================
 *  COMPILER SETTINGS
 *====================*/

#define LV_BIG_ENDIAN_SYSTEM  0
#define LV_ATTRIBUTE_TICK_INC
#define LV_ATTRIBUTE_TIMER_HANDLER
#define LV_ATTRIBUTE_FLUSH_READY
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1
#define LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_LARGE_CONST
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY
#define LV_ATTRIBUTE_FAST_MEM
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning
#define LV_USE_FLOAT 0
#define LV_USE_MATRIX 0

/*==================
 *   FONT USAGE
 *==================*/

#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 0
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0

#define LV_FONT_DEFAULT &lv_font_montserrat_16

#define LV_FONT_FMT_TXT_LARGE   0
#define LV_USE_FONT_COMPRESSED   0
#define LV_USE_FONT_SUBPX       0
#if LV_USE_FONT_SUBPX
    #define LV_FONT_SUBPX_BGR 0
#endif
#define LV_USE_FONT_PLACEHOLDER 1

/*==================
 *  TEXT SETTINGS
 *==================*/

#define LV_TXT_ENC LV_TXT_ENC_UTF8
#define LV_TXT_BREAK_CHARS " ,.;:-_)]}"
#define LV_TXT_LINE_BREAK_LONG_LEN 0
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/*==================
 *  WIDGETS
 *==================*/

#define LV_WIDGETS_HAS_DEFAULT_VALUE 1
#define LV_USE_ANIMIMG    1
#define LV_USE_ARC        1
#define LV_USE_BAR        1
#define LV_USE_BUTTON     1
#define LV_USE_BUTTONMATRIX 1
#define LV_USE_CALENDAR   1
#define LV_USE_CANVAS     1
#define LV_USE_CHART      1
#define LV_USE_CHECKBOX   1
#define LV_USE_DROPDOWN   1
#define LV_USE_IMAGE      1
#define LV_USE_IMAGEBUTTON 1
#define LV_USE_KEYBOARD   1
#define LV_USE_LABEL      1
#define LV_USE_LED        1
#define LV_USE_LINE       1
#define LV_USE_LIST       1
#define LV_USE_LOTTIE     0
#define LV_USE_MENU       1
#define LV_USE_MSGBOX     1
#define LV_USE_ROLLER     1
#define LV_USE_SCALE      1
#define LV_USE_SLIDER     1
#define LV_USE_SPAN       1
#define LV_USE_SPINBOX    1
#define LV_USE_SPINNER    1
#define LV_USE_SWITCH     1
#define LV_USE_TABLE      1
#define LV_USE_TABVIEW    1
#define LV_USE_TEXTAREA   1
#define LV_USE_TILEVIEW   1
#define LV_USE_WIN        1

/*==================
 * THEMES
 *==================*/

#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_USE_THEME_SIMPLE  1
#define LV_USE_THEME_MONO    0

/*==================
 * LAYOUTS
 *==================*/

#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*====================
 * 3RD PARTY LIBRARIES
 *====================*/

/* Filesystem: POSIX API for SD_MMC (ESP-IDF VFS mount at /sdcard) */
#define LV_FS_DEFAULT_DRIVER_LETTER '\0'

#define LV_USE_FS_STDIO  0
#define LV_USE_FS_POSIX  1
#if LV_USE_FS_POSIX
    #define LV_FS_POSIX_LETTER 'S'
    #define LV_FS_POSIX_PATH "/sdcard"
    #define LV_FS_POSIX_CACHE_SIZE 1024
#endif

#define LV_USE_FS_WIN32 0
#define LV_USE_FS_FATFS 0
#define LV_USE_FS_MEMFS 0
#define LV_USE_FS_LITTLEFS 0
#define LV_USE_FS_ARDUINO_ESP_LITTLEFS 0
#define LV_USE_FS_ARDUINO_SD 0
#define LV_USE_FS_UEFI  0
#define LV_USE_FS_FROGFS 0

/* Image decoders */
#define LV_USE_LODEPNG  1   /* PNG decoder (built-in, no external lib) */
#define LV_USE_LIBPNG   0
#define LV_USE_BMP      1   /* BMP decoder */
#define LV_USE_TJPGD    1   /* TinyJPG decoder (built-in, low memory) */
#define LV_USE_LIBJPEG_TURBO 0
#define LV_USE_LIBWEBP  0
#define LV_USE_GIF      0
#define LV_USE_GSTREAMER 0
#define LV_BIN_DECODER_RAM_LOAD 1  /* Decode LVGL .bin images to RAM (PSRAM) */
#define LV_USE_RLE      0
#define LV_USE_QRCODE   0
#define LV_USE_BARCODE  0
#define LV_USE_FREETYPE 0
#define LV_USE_TINY_TTF 0
#define LV_USE_RLOTTIE  0

/* Vector graphics */
#define LV_USE_VECTOR_GRAPHIC 0

/* Others */
#define LV_USE_SNAPSHOT     0
#define LV_USE_MONKEY       0
#define LV_USE_GRIDNAV      0
#define LV_USE_FRAGMENT     0
#define LV_USE_IMGFONT      0
#define LV_USE_OBSERVER     1
#define LV_USE_IME_PINYIN   0
#define LV_USE_FILE_EXPLORER 0
#define LV_USE_PROFILER     0

#endif /* LV_CONF_H */
