/********************************************************************************
 * This code is used for robot control
*********************************************************************************
 * Author        Data            Version
 * Liren         2019/01/18      1.0
********************************************************************************/
#ifndef __ROC_LOG_H
#define __ROC_LOG_H

#include <stdint.h>
#include "RocError.h"


#define ROC_MAX_BUFFER_SIZE         120

#define ROC_FONT_BLACK              "\033[0;30m"
#define ROC_FONT_RED                "\033[0;31m"
#define ROC_FONT_GREEN              "\033[0;32m"
#define ROC_FONT_YELLOW             "\033[0;33m"
#define ROC_FONT_BLUE               "\033[0;34m"
#define ROC_FONT_PINK               "\033[0;35m"
#define ROC_FONT_DARK_GREEN         "\033[0;36m"
#define ROC_FONT_WHITE              "\033[0;37m"

#define ROC_CLOSE_PROPERTY          "\033[0m"
#define ROC_HELIGHT_PROPERTY        "\033[1m"
#define ROC_UNDERLINE_PROPERTY      "\033[4m"
#define ROC_FLICKER_PROPERTY        "\033[5m"
#define ROC_NEGATE_PROPERTY         "\033[7m"
#define ROC_BLANKING_PROPERTY       "\033[8m"
#define ROC_UP_MOVE_PROPERTY        "\033[nAm"
#define ROC_DOWN_MOVE_PROPERTY      "\033[nBm"
#define ROC_RIGHT_MOVE_PROPERTY     "\033[nCm"
#define ROC_LEFT_MOVE_PROPERTY      "\033[nDm"

#define ROC_HELIGHT_SETTING         "\033[1m\033[47;37m"

#define ROC_BACK_BLACK              40
#define ROC_BACK_RED                41
#define ROC_BACK_GREEN              42
#define ROC_BACK_YELLOW             43
#define ROC_BACK_BLUE               44
#define ROC_BACK_PINK               45
#define ROC_BACK_DARK_GREEN         46
#define ROC_BACK_WHITE              47


ROC_RESULT RocLogI(const char *function, uint32_t line, const char *fmt, ...);
ROC_RESULT RocLogW(const char *function, uint32_t line, const char *fmt, ...);
ROC_RESULT RocLogE(const char *function, uint32_t line, const char *fmt, ...);
ROC_RESULT RocLogN(const char *function, uint32_t line, const char *fmt, ...);


#define ROC_LOGI(fmt, ...)      RocLogI(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define ROC_LOGW(fmt, ...)      RocLogW(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define ROC_LOGE(fmt, ...)      RocLogE(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define ROC_LOGN(fmt, ...)      RocLogN(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)


#endif
