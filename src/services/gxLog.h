#ifndef GX_LOG_H
#define GX_LOG_H

#include <Arduino.h>

//-------------------------------------------------------------------------------
// Log levels
// NOTE: Values are intentionally non-contiguous (gaps at 2, 6, 8) to leave
//       room for future intermediate levels. Filtering uses "level < item.level".
//-------------------------------------------------------------------------------
#define GXLOG_LVL_NONE    9   // logging for this tag disabled
#define GXLOG_LVL_ERROR   7
#define GXLOG_LVL_WARN    5
#define GXLOG_LVL_SYSTEM  4
#define GXLOG_LVL_INFO    3
#define GXLOG_LVL_DEBUG   1
#define GXLOG_LVL_TRACE   0

//-------------------------------------------------------------------------------
// Output options (bit flags, combine with OR)
//-------------------------------------------------------------------------------
#define GXLOG_OPT_TIMESTAMP      0x0001
#define GXLOG_OPT_ANSI_ESC       0x0002
#define GXLOG_OPT_ITEM_COLOR     0x0004

//       not yet implemented in gxLog() - reserved for future use.
//       Will require __FILE__ / __func__ / __LINE__ to be passed into the
//       log macros below.
#define GXLOG_OPT_FILENAME       0x0010
#define GXLOG_OPT_FUNCTIONNAME   0x0020
#define GXLOG_OPT_LINENUMBER     0x0040

#if 0

#define xLog_E(xtag, fmt, args...)     do { if (XLOG_LVL<=XLOG_LVL_ERROR) { printf("(%6d)[E][%04s] : " fmt, millis(), xtag, ## args); printf("\r\n"); } } while (0)
#define xLog_W(xtag, fmt, args...)     do { if (XLOG_LVL<=XLOG_LVL_WARN)  { printf("(%6d)[W][%04s] : " fmt, millis(), xtag, ## args); printf("\r\n"); } } while (0)
#define xLog_I(xtag, fmt, args...)     do { if (XLOG_LVL<=XLOG_LVL_INFO)  { printf("(%6d)[I][%04s] : " fmt, millis(), xtag, ## args); printf("\r\n"); } } while (0)
#define xLog_D(xtag, fmt, args...)     do { if (XLOG_LVL<=XLOG_LVL_DEBUG) { printf("(%6d)[D][%04s] : " fmt, millis(), xtag, ## args); printf("\r\n"); } } while (0)

#else

//-------------------------------------------------------------------------------
// Log macros
// NOTE: "args..." and "## args" are a GCC extension. Works fine on the
//       ESP32 toolchain; not strictly ISO C++. The xLog_* and gxLog_*
//       families are functionally identical (kept for API compatibility).
//-------------------------------------------------------------------------------
#define xLog_E(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_ERROR, fmt,## args);  } while (0)
#define xLog_W(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_WARN, fmt,## args);   } while (0)
#define xLog_S(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_SYSTEM, fmt,## args); } while (0)
#define xLog_I(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_INFO, fmt,## args);   } while (0)
#define xLog_D(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_DEBUG, fmt,## args);  } while (0)

#define gxLog_E(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_ERROR, fmt,## args);  } while (0)
#define gxLog_W(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_WARN, fmt,## args);   } while (0)
#define gxLog_S(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_SYSTEM, fmt,## args); } while (0)
#define gxLog_I(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_INFO, fmt,## args);   } while (0)
#define gxLog_D(xtag, fmt, args...)     do { gxLog(xtag, GXLOG_LVL_DEBUG, fmt,## args);  } while (0)

#endif

//-------------------------------------------------------------------------------
// Public API
//-------------------------------------------------------------------------------
void gxLog(const char *tag, uint16_t level, const char *fmt, ...);
void gxLog_enable();
void gxLog_disable();
void gxLog_setLevel(const char *tag, uint16_t level);
void gxLog_setColor(const char *tag, const char *ansi_esc_color);

void gxLog_setOptions(uint16_t options);
void gxLog_clearOptions(uint16_t options);

#define gxPrintf(fmt, args...) do { printf("" fmt, ## args); } while (0)
// NOTE: gxPrintf writes via printf() (stdout), while gxLog() writes via
//       Serial. Depending on the build config these can be different
//       channels. Unify if consistent routing is required.

//-------------------------------------------------------------------------------
// ANSI escape sequences
// NOTE: \033 and \x1B are the same ESC character; mixed notation kept as-is.
//-------------------------------------------------------------------------------
#define ANSI_ESC_BELL       "\x07"
#define ANSI_ESC_OFF        "\033[0m"
#define ANSI_ESC_BOLD       "\033[1m"
#define ANSI_ESC_LOWINTENS  "\033[2m"
#define ANSI_ESC_UNDERLINE  "\033[4m"
#define ANSI_ESC_BLINK      "\033[5m"
#define ANSI_ESC_REVERSE    "\033[7m"

#define ANSI_ESC_FG_BLACK   "\x1B[30m"
#define ANSI_ESC_FG_RED     "\x1B[31m"
#define ANSI_ESC_FG_GREEN   "\x1B[32m"
#define ANSI_ESC_FG_YELLOW  "\x1B[33m"
#define ANSI_ESC_FG_BLUE    "\x1B[34m"
#define ANSI_ESC_FG_MAGENTA "\x1B[35m"
#define ANSI_ESC_FG_CYAN    "\x1B[36m"
#define ANSI_ESC_FG_WHITE   "\x1B[37m"

#define ANSI_ESC_FG_BRIGHT_GREY    "\033[90m"
#define ANSI_ESC_FG_BRIGHT_RED     "\033[91m"
#define ANSI_ESC_FG_BRIGHT_GREEN   "\033[92m"
#define ANSI_ESC_FG_BRIGHT_YELLOW  "\033[93m"
#define ANSI_ESC_FG_BRIGHT_BLUE    "\033[94m"
#define ANSI_ESC_FG_BRIGHT_PURPLE  "\033[95m"
#define ANSI_ESC_FG_BRIGHT_CYAN    "\033[96m"
#define ANSI_ESC_FG_BRIGHT_WHITE   "\033[97m"

#define ANSI_ESC_BG_BLACK   "\x1B[40m"
#define ANSI_ESC_BG_RED     "\x1B[41m"
#define ANSI_ESC_BG_GREEN   "\x1B[42m"
#define ANSI_ESC_BG_YELLOW  "\x1B[43m"
#define ANSI_ESC_BG_BLUE    "\x1B[44m"
#define ANSI_ESC_BG_MAGENTA "\x1B[45m"
#define ANSI_ESC_BG_CYAN    "\x1B[46m"
#define ANSI_ESC_BG_WHITE   "\x1B[47m"

#endif