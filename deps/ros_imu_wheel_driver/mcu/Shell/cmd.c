#include "cmd.h"
#include "read-line.h"
#include "syslog.h"

#include <string.h>
#include "main.h"
#include "config.h"

static const cmd_t cmd_array[];

uint8_t ucCmd_exec(uint8_t argc, char **argv)
{
	static uint32_t ulNFcnt = 0;
	static uint32_t ulEPcnt = 0;
  uint8_t i;
	
	if (argc != 0 && argv != NULL)
	{
		for ( i=0; cmd_array[i].cmd_name != NULL; i++)
		{
			if(strcmp(cmd_array[i].cmd_name, argv[0]) == 0)
			{
				if(cmd_array[i].cmd_func != NULL)
				{
					//TODO:write log
					(cmd_array[i].cmd_func)(argc, argv);
					return 0;
				}
			}
		}
		ts_printf("没有找到这条命令，请检查你的输入或输入\"help\"查看所有命令以及帮助信息。\r\n");
		ulNFcnt++;
		return 1;
	}
	else
	{
		ulEPcnt++;
		return 1;
	}
}



static uint8_t prvCmd_printarg(uint8_t argc, char **argv)
{
	uint8_t i;
	for ( i=0; i<argc; i++)
	{
		ts_printf("%d: %s \r\n", i, argv[i]);
	}
	return 0;
}

static uint8_t prvCmd_help(uint8_t argc, char **argv)
{
	uint8_t i;
	(void) argc;
	(void) argv;

	ts_printf("------------------------------------------------------------------\r\n");
	for (i=0; cmd_array[i].cmd_name != NULL; i++)
	{
		ts_printf("    %s  %s \r\n", cmd_array[i].cmd_name, cmd_array[i].cmd_help);
	}
	ts_printf("------------------------------------------------------------------\r\n");
	return 0;
}
#if 0
static uint8_t prvCmd_setpro(uint8_t argc, char **argv)
{
	if (argc == 1)
	{
		ucChangePrompt("\0");
	}
	else if (argc == 2)
	{
		ucChangePrompt(argv[1]);
	}
	else
	{
		ts_printf("参数格式错误！\r\n");
	}
	return 0;
}



static uint8_t prvCmd_echo(uint8_t argc, char **argv)
{
	if (argc == 2)
	{
		if(strcmp("-on", argv[1]) == 0)
		{
			ucChangeEcho(0x01);
		}
		if(strcmp("-off", argv[1]) == 0)
		{
			ucChangeEcho(0x00);
		}
	}
	else
	{
		ts_printf("参数格式错误！\r\n");
	}
	return 0;
}
#endif
//---------------------------------------------------------------------
//freertos相关函数 查看
static uint8_t prvCmd_Version(uint8_t argc, char **argv)
{
	(void) argc;
	(void) argv;
	ts_printf("------------------------------------------------------------------\r\n");
	ts_printf(" \r\n%s :HardVersion %s,SoftVersion %s\r\n",P_NAME,HAEDVER,HAEDVER);	
	ts_printf("------------------------------------------------------------------\r\n");
	return 0;
}
static uint8_t prvCmd_List(uint8_t argc, char **argv)
{
	(void) argc;
	(void) argv;
	return 0;
}

////////////////////////////用户配置区///////////////////////////////////////////////


//---------------------------------------------------------------------
static const cmd_t cmd_array[] =
{
	{"printarg", "  --打印出你输入的所有参数。", prvCmd_printarg},
	{"help", "      --打印出所有的命令以及提示信息。", prvCmd_help},
//	{"setpro", "    --修改命令行提示信息。 参数：Shell: ", prvCmd_setpro},
//	{"echo", "      --开启或关闭字符回显模式，建议开启。\
//参数：-on 打开回显模式， -off 关闭回显模式。", prvCmd_echo},
	{"ver", "       --打印出系统版本信息。", prvCmd_Version},
	{"list", "      --打印出线程使用情况。", prvCmd_List},
	{NULL, NULL, NULL}
};
