/** 
* @file     universe.cpp
* @brief    用于嵌入式C++工程的补充文件
* @details  在嵌入式工程使用C++时，有一些问题需要处理来使C++更好的适用于工程 \n
*           本文件局限性较大，最佳运行情况为使用keil为编译环境且使用FreeRTOS
* @author   WMD
* @date     2018年10月26日22:06:00
* @version  0.1
* @par Copyright (c):  
*       WMD 
* @par 日志
*				2019.11.29 为适应代码规范，修改了此文件名为：bsp_universe.cpp
*/  
#include <rt_sys.h>
#include "bsp_car_config.hpp"
#include "freertos.h"
#include <time.h>
using namespace std;
#ifdef USE_OWM_SYS_DEF
extern "C" {
#pragma import(__use_no_semihosting_swi)  
#pragma import(_main_redirection)  
const char __stdin_name[150]; //让它存在就行了 
const char __stdout_name[150];  
const char __stderr_name[150];  
typedef int FILEHANDLE;  

//重写标准库函数，这时printf、fopen、fclose等文件操作函数运行时就会调用你的重写函数，这些重写函数只是几个简单的例子，并没有重写所有的文件操作函数  
void _sys_exit(int status)  
{  
    while(1);  
}  
FILEHANDLE _sys_open(const char *name, int openmode)  
{  
    return 0;  
}  

int _sys_close(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)  
{  
    return 0;  
}  

int _sys_read(FILEHANDLE fh, unsigned char*buf, unsigned len, int mode)  
{  
    return 0;  
}  

int _sys_istty(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_seek(FILEHANDLE fh, long pos)  
{  
    return 0;  
}  

int _sys_ensure(FILEHANDLE fh)  
{  
    return 0;  
}  

long _sys_flen(FILEHANDLE fh)  
{  
    return 0;  
}  

int _sys_tmpnam(char *name, int fileno, unsigned maxlength)  
{  
    return 0;  
}  

void _ttywrch(int ch)  
{  
}
time_t time(time_t *t)  
{  
    return 0;  
}  
int remove(const char *filename)  
{  
    return 0;  
}  

char *_sys_command_string(char *cmd, int len)  
{  
    return 0;  
}  

clock_t clock(void)  
{  
    return 0;  
}  
///重定向的一个很重要函数，如有需要请在外部覆盖定义
__attribute__((weak)) int fputc(int ch, FILE* f)
{
    return ch;
}
}//C与C++的分界线
#endif

/** 
* @brief  重载new运算符使其可以使用操作系统的内存调度管理
* @warning pvPortMalloc()函数需要操作系统支持
* @par 日志 
*
*/
void *operator new (size_t size)
{
  void* p=pvPortMalloc(size);
	if(p==NULL)while(1);
	return p;
}
/** 
* @brief  重载new运算符使其可以使用操作系统的内存调度管理
* @warning pvPortMalloc()函数需要操作系统支持
* @par 日志 
*
*/
void*
operator new[](size_t size)
{
  return operator new(size);
}
/** 
* @brief  重载delete运算符使其可以使用操作系统的内存调度管理
* @warning vPortFree()函数需要操作系统支持
* @par 日志 
*
*/
void operator delete(void* p)
{
	vPortFree(p);
}
/** 
* @brief  重载delete运算符使其可以使用操作系统的内存调度管理
* @warning vPortFree()函数需要操作系统支持
* @par 日志 
*
*/
void
operator delete[](void* pointer)
{
  operator delete(pointer);
}
