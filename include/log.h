/**
 * @file app_log.h
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief this provides logging formats
 * @version 0.1
 * @date 2018-12-18
 *
 * @copyright Copyright (c) 2018
 *
 */
#pragma once

#include <iomanip>
#include <iostream>

#define log_com(c)                                                             \
  (std::cout << "[" c "] " << __FILE__ << ":" << __LINE__ << " [" << __func__  \
             << "()] ")

#if 1
#define logd log_com("D")
#else
#define logd
#endif

#if 1
#define logi log_com("I")
#else
#define logi
#endif

#if 1
#define logw log_com("W")
#else
#define logw
#endif

#if 1
#define loge log_com("E")
#else
#define loge
#endif
