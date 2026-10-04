//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
//
//   xlog
//     - for debugging and logging
//
//-------------------------------------------------------------------------------
//   date:  27.03.2025
//   autor: Gonzo
//-------------------------------------------------------------------------------
//
//
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
#include <Arduino.h>
#include "gxList.h"
#include "gxLog.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//--------------------------------------------------------------
typedef struct{
    String tag;
    uint16_t level;
    char color[5] = {0};
}gxLogItem_t;

//--------------------------------------------------------------
gxList<gxLogItem_t *> _itemList;

const uint16_t BUFFSIZE = 512;
const uint16_t MUTEX_TIMEOUT = (100 / portTICK_PERIOD_MS);

SemaphoreHandle_t mtx_xLog   = xSemaphoreCreateMutex();

uint16_t _options = GXLOG_OPT_TIMESTAMP;
bool _enabled = true;

gxLogItem_t *_xLog_getItem(String tag);


//------------------------------------------------------------------------
String _formatToString(const char *fmt, ...)
{
static char buf[BUFFSIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, BUFFSIZE, fmt, args);
    va_end(args);

return(String(buf));
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog(const char *tag, uint16_t level, const char *fmt, ...)
{
bool found = false;
gxLogItem_t *xlu;
static char buf[BUFFSIZE];
uint32_t xlogtime;
String msg;

    if( !_enabled )
        return;

    xSemaphoreTake( mtx_xLog, MUTEX_TIMEOUT );

    xlu = _xLog_getItem(tag);

    //--- level ERROR
    if( xlu->level == GXLOG_LVL_NONE ){
        xSemaphoreGive( mtx_xLog );
        return;
    }

    xlogtime = millis();

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, BUFFSIZE, fmt, args);
    va_end(args);

    //--- check if the tag is enabled
    if( level < xlu->level ) {
        xSemaphoreGive( mtx_xLog );
        return;
    }

    //--- check if timestamp-option is set
    if( _options & GXLOG_OPT_TIMESTAMP ) {
        msg += _formatToString("(%d.%03d) ", xlogtime/1000, xlogtime%1000);
    }

    //--- check if color-option is set
    if( _options & GXLOG_OPT_ITEM_COLOR )
        if(( level != GXLOG_LVL_ERROR ) && ( level != GXLOG_LVL_WARN ))
            if(xlu->color[0] != 0)
                msg += _formatToString("%s", xlu->color);

    //--- level ERROR
    if( level == GXLOG_LVL_ERROR )
        if( _options & GXLOG_OPT_ITEM_COLOR ) msg += _formatToString("%s[%-8s] -ERROR- %s%s", ANSI_ESC_FG_RED, tag, buf, ANSI_ESC_OFF);
        else                                  msg += _formatToString("[%-8s] -ERROR- %s", tag, buf);

    //--- level WARN
    else if( level == GXLOG_LVL_WARN )
        if( _options & GXLOG_OPT_ITEM_COLOR )
            msg += _formatToString( "%s[%-8s] -WARN- %s%s", ANSI_ESC_FG_YELLOW, tag, buf, ANSI_ESC_OFF);
        else
            msg += _formatToString( "[%-8s] -WARN- %s", tag, buf);

    //--- level INFO
    else if( (level == GXLOG_LVL_SYSTEM) || (level == GXLOG_LVL_INFO) || (level == GXLOG_LVL_DEBUG) )
        msg += _formatToString( "[%-8s] %s", tag, buf);

    //-----------

    // //--- clear ansii-esc
    // if( _options & GXLOG_OPT_ANSI_ESC )
    //     msg += String(ANSI_ESC_OFF);

    //--- reset color
    if( _options & GXLOG_OPT_ITEM_COLOR )
        if(xlu->color[0] != 0)
            msg += String(ANSI_ESC_OFF);

    msg += String("\r\n");

    printf("%s", msg.c_str());
    Serial.flush();

    xSemaphoreGive( mtx_xLog );
}


//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog_setLevel(const char *tag, uint16_t level)
{
gxLogItem_t *xlu;

    xSemaphoreTake( mtx_xLog, MUTEX_TIMEOUT );

    xlu = _xLog_getItem(tag);
    xlu->level = level;

    xSemaphoreGive( mtx_xLog );
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog_setColor(const char *tag, const char *ansi_esc_color)
{
gxLogItem_t *xlu;

    xSemaphoreTake( mtx_xLog, MUTEX_TIMEOUT );

    xlu = _xLog_getItem(tag);
    strcpy(xlu->color, ansi_esc_color);

    xSemaphoreGive( mtx_xLog );
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog_setOptions(uint16_t opt)
{
    _options = _options | opt;
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog_clearOptions(uint16_t opt)
{
    _options = _options & ~opt;
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
void gxLog_enable()  { _enabled = true; }
void gxLog_disable() { _enabled = false; }

//------------------------------------------------------------------------
//------------------------------------------------------------------------
gxLogItem_t *_xLog_getItem(String unit)
{
gxLogItem_t *xlu;
bool found = false;

    for(int n=0; n<_itemList.size(); n++) {
        xlu = _itemList.get(n);
        if( xlu->tag == unit ) {
            found = true;
            break; }
        }

    if( !found ) {
        xlu = new gxLogItem_t();
        xlu->tag = unit;
        xlu->level = GXLOG_LVL_INFO;
        _itemList.add(xlu);  }

    return(xlu);
}


// void gxLog(const char *tag, uint16_t level, const char *fmt, ...) {};
// void gxLog_enable() {};
// void gxLog_disable() {};
// void gxLog_setLevel(const char *tag, uint16_t level) {};
// void gxLog_setColor(const char *tag, const char *ansi_esc_color) {};

// void gxLog_setOptions(uint16_t options) {};
// void gxLog_clearOptions(uint16_t options){};
