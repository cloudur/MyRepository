/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "batteryMonitor.h"

#include <linux/input.h>
#include <linux/netlink.h>
#include <errno.h>
#include <libgen.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <time.h>
#include <fcntl.h>
#include <dirent.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>

#include "events.h"
#include "uevent.h"


void healthd_mode_charger_init(struct healthd_config *config);
int healthd_mode_charger_preparetowait(void);
void healthd_mode_charger_heartbeat(void);
void healthd_mode_charger_battery_update(
		struct BatteryProperties *props);
void healthd_battery_update(void);



// epoll_create() parameter is actually unused
#define MAX_EPOLL_EVENTS 40
static int g_uevent_fd = -1;
static int g_wakealarm_fd = -1;

// Periodic chores intervals in seconds
#define DEFAULT_PERIODIC_CHORES_INTERVAL_FAST (60 * 10)
#define DEFAULT_PERIODIC_CHORES_INTERVAL_SLOW (60 * 10)
static int wakealarm_wake_interval = DEFAULT_PERIODIC_CHORES_INTERVAL_FAST;

// -1 for no epoll timeout
static int g_awake_poll_interval = -1;
static int g_eventct;
static int g_epollfd;
static int g_online_check = -1;
static BatteryMonitor* gBatteryMonitor;
struct healthd_mode_ops *ghealthd_mode_ops;
static struct charger g_charger_state;
static int g_ltud_sock = -1;
int g_nJustforDebugLed = 0;

#define UEVENT_MSG_LEN 2048
#define MSEC_PER_SEC            (1000LL)
#define NSEC_PER_MSEC           (1000000LL)
#define POWER_SUPPLY_SUBSYSTEM  "power_supply"
#define CHARGING_ENABLED_PATH   "/sys/class/power_supply/battery/charging_enabled"
#define USB_ONLINE_PATH         "/sys/class/power_supply/usb/online"
#define BATTERY_CAPATITY_PATH	"/sys/class/power_supply/battery/capacity"
#define BATTERY_STATUS_PATH     "/sys/class/power_supply/battery/status"
#define LTUD_SOCKET "/tmp/UNIX.domain"

//#define SHUTDOWN_COMMAND "/sbin/shutdown -P -t 1 time \"1 second\""
#define SHUTDOWN_PATH    "/sbin/shutdown"
//#define SHUTDOWN_COMMAND "shutdown -h -P -time \"now\""
#define SHUTDOWN_COMMAND "shutdown -h now"
#define RM_COMMAND       "busybox rm -rf /data/test"

//add for debug
#define LOG_DEBUG_FILE_PATH "/data/charge_debug_log"
unsigned char g_chDebugMask = 1;//0-not debug, 1-debug
#define LOG_MASK (1 << 0)
#define DEBUG_LOG(...) \
	do {if(g_chDebugMask & LOG_MASK) \
		printf(__VA_ARGS__);\
		FILE *pFdDebug = fopen(LOG_DEBUG_FILE_PATH, "a+");\
		if(NULL != pFdDebug){\
			char *pChTemp = (char*)malloc(sizeof(char) * 512);\
			if(NULL != pChTemp){\
				sprintf(pChTemp, __VA_ARGS__);\
				fputs(pChTemp, pFdDebug);\
				free(pChTemp);\
				fclose(pFdDebug);}}\
	}while(0)
//#define DEBUG_LOG(...) \
//  printf(__VA_ARGS__)
//#define DEBUG_USB_CABLE


//add for fota
#define USB_CABLE_PLUG_IN_COUNT_TIMES_DURING 3
int g_nUsbCablePlugInCount = 0;

//#define FOTA_COMMAND_SYMLINK              "/bin/ln -s da fota_upgrade"
#define FOTA_COMMAND_SYMLINK              "/bin/ln -s /sbin/da /sbin/fota_upgrade"
#define FOTA_COMMAND_PATH                 "/sbin/fota_upgrade"
#define FOTA_COMMAND                      "fota_upgrade"
#define FOTA_PROCESS_RUNNING_JUDGE         "ps -ef | grep \"fota_upgrade\" | awk '{print $4'}"
#define FOTA_UPGRADE_RESULT_PATH          "/cache/result"

#define NETWORK_OPERATE_COMMAND           "/usr/tests/dsi_netctrl_test fota"
#define NETWORK_CONNECTING_RESULT_PATH    "/data/network"
#define NETWORK_CONNECTING_RESULT_EVENT   "EVENT: DSI_EVT_NET_IS_CONN"
#define NETWORK_DNS_CONFIG_COMMAND        "udhcpc -i rmnet_data0  -s /etc/udhcpc.d/50default"
#define NETWORK_DNS_CONFIG_RESULT_STRING  "Adding DNS "
#define NETWORK_DNS_CONFIG_VERIFY_COMMAND "/sbin/route | grep \"default\" | awk '{print $2}'"


//#define DEVICES_SERIAL_NUMBER   "/sys/devices/soc0/serial_number"
#define DEVICES_SERIAL_NUMBER 	  "/data/imei" //allen
//#define DEVICES_VERSION         "/proc/version"
#define DEVICES_VERSION         "/build.prop" //allen
#define DEVICES_BRAND           "/sys/devices/soc0/machine"
#define DEVICES_MODEL           "LTUB"
#define FOTA_POLL_INTERVAL_TIME      90
#define FOTA_UPGRADE_LIMITED_SOC     40 //80 modify to 40
#define DEVICES_SERIAL_NUMBER_LENGTH 16
#define DEVICES_VERSION_LENGTH       128
#define DEVICES_BRAND_LENGTH         16
#define DEVICES_MODEL_LENGTH         16

typedef struct FotaDeviceInfo
{
	char chDevicesSerialNumber[DEVICES_SERIAL_NUMBER_LENGTH];
	char chDevicesVersion[DEVICES_VERSION_LENGTH];
	char chDevicesBrand[DEVICES_BRAND_LENGTH];
	char chDevicesModel[DEVICES_MODEL_LENGTH];

	bool bDevicesIsUpgrading;
	bool bDevicesHasReportedResult;
	bool bNetWorkConnected;
};

struct FotaDeviceInfo *g_pFotaDeviceInfo = NULL;

static struct healthd_config g_healthd_config = {
	.periodic_chores_interval_fast = DEFAULT_PERIODIC_CHORES_INTERVAL_FAST,
	.periodic_chores_interval_slow = DEFAULT_PERIODIC_CHORES_INTERVAL_SLOW,
	.batteryStatusPath = string(""),
	.batteryHealthPath = string(""),
	.batteryPresentPath = string(""),
	.batteryCapacityPath = string(""),
	.batteryVoltagePath = string(""),
	.batteryTemperaturePath = string(""),
	.batteryTechnologyPath = string(""),
	.batteryCurrentNowPath = string(""),
	.batteryCurrentAvgPath = string(""),
	.batteryChargeCounterPath = string(""),
	//.energyCounter = NULL,
	.boot_min_cap = 0,
	//.screen_on = NULL,
};

static struct healthd_mode_ops charger_ops = {
	.init = healthd_mode_charger_init,
	.preparetowait = healthd_mode_charger_preparetowait,//should deleted
	.heartbeat = healthd_mode_charger_heartbeat,//should deleted
	.battery_update = healthd_mode_charger_battery_update,
};

/* current time in milliseconds */
static int64_t curr_time_ms(void)
{
	struct timespec tm;
	clock_gettime(CLOCK_MONOTONIC, &tm);
	return tm.tv_sec * MSEC_PER_SEC + (tm.tv_nsec / NSEC_PER_MSEC);
}

static int read_file(const char *path, char *buf, size_t sz)
{
	int fd;
	size_t cnt;

	fd = open(path, O_RDONLY, 0);
	if (fd < 0)
		goto err;

	cnt = read(fd, buf, sz - 1);
	if (cnt <= 0)
		goto err;
	buf[cnt] = '\0';
	if (buf[cnt - 1] == '\n') {
		cnt--;
		buf[cnt] = '\0';
	}

	close(fd);
	return cnt;

err:
	if (fd >= 0)
		close(fd);
	return -1;
}

static int read_file_int(const char *path, int *val)
{
	char buf[32];
	int ret;
	int tmp;
	char *end;

	ret = read_file(path, buf, sizeof(buf));
	if (ret < 0)
		return -1;

	tmp = strtol(buf, &end, 0);
	if (end == buf ||
			((end < buf+sizeof(buf)) && (*end != '\n' && *end != '\0')))
		goto err;

	*val = tmp;
	return 0;

err:
	return -1;
}

bool check_file_exist(const char *path) {
	struct stat buffer;
	int err = stat(path, &buffer);

	if(err == 0)
		return true;
	if(errno == ENOENT) {
		DEBUG_LOG("file: %s  do not exist , %s\n", path, strerror(errno));
		return false;
	}
	return false;
}

int clear_file(const char *filepath) {
	FILE *fp = NULL;

	/*open as Write, and then close, mean clear file content */
	fp = fopen(filepath, "w");
	if(!fp)
		return -1;

	fclose(fp);
	remove(filepath);
	return 0;
}

bool fota_init_devices_information()
{
	bool nRet = true;
	char i,j;
	g_pFotaDeviceInfo = (struct FotaDeviceInfo *)malloc(sizeof(struct FotaDeviceInfo));

	if(g_pFotaDeviceInfo)
	{
		memset(g_pFotaDeviceInfo, 0, sizeof(struct FotaDeviceInfo));

		//Serial Number
		unsigned int uCount = read_file(DEVICES_SERIAL_NUMBER, g_pFotaDeviceInfo->chDevicesSerialNumber, DEVICES_SERIAL_NUMBER_LENGTH);
		if(uCount <= 0)
		{
			nRet &= false;
			DEBUG_LOG("%s, Can't get devices serial number.\n", __func__);
		}
		else
		{
			DEBUG_LOG("%s, Devices SerialNumber is %s, count=%d.\n",__func__,
					g_pFotaDeviceInfo->chDevicesSerialNumber, uCount);
		}

		//version
		uCount = read_file(DEVICES_VERSION, g_pFotaDeviceInfo->chDevicesVersion, DEVICES_VERSION_LENGTH);
		if(uCount <= 0)
		{
			nRet &= false;
			DEBUG_LOG("%s, Can't get devices Version.\n", __func__);
		}
		else
		{
			//del superfluous information, ro.build.version.release=201611080008 -> 201611080008
			for (i = 0; i < uCount; i++)
			{
				if (*(g_pFotaDeviceInfo->chDevicesVersion + i) == '=')
				{
					i++;
					break;
				}
			}
			uCount -= i;
			for (j = 0; j < uCount; j++)
			{
				*(g_pFotaDeviceInfo->chDevicesVersion+j) = *(g_pFotaDeviceInfo->chDevicesVersion+i+j);
			}
			*(g_pFotaDeviceInfo->chDevicesVersion+j) = 0;

			DEBUG_LOG("%s, Devices Version is %s, count=%d.\n", __func__,
					g_pFotaDeviceInfo->chDevicesVersion, uCount);
		}

		//brand
		uCount = read_file(DEVICES_BRAND, g_pFotaDeviceInfo->chDevicesBrand, DEVICES_BRAND_LENGTH);
		if(uCount <= 0)
		{
			nRet &= false;
			DEBUG_LOG("%s, Can't get devices brand.\n", __func__);
		}
		else
		{
			DEBUG_LOG("%s, Devices brand is %s, count=%d.\n", __func__,
					g_pFotaDeviceInfo->chDevicesBrand, uCount);
		}

		//model
		uCount = sprintf(g_pFotaDeviceInfo->chDevicesModel, "%s", DEVICES_MODEL);
		if(uCount <= 0)
		{
			nRet &= false;
			DEBUG_LOG("%s, Can't get devices model.\n", __func__);
		}
		else
		{
			DEBUG_LOG("%s, Devices model is %s, count=%d.\n", __func__,
					g_pFotaDeviceInfo->chDevicesModel, uCount);
		}
	}
	else
	{
		DEBUG_LOG("%s, Can't malloc space for Fota information struct.\n", __func__);
	}

	return nRet;
}


void fota_local_exe_cmd(const char *cmd, char *result, int size)
{
	char line[1024];
	char ps[64] = { 0 };
	FILE *pp;

	sprintf(ps, "%s", cmd);
	if((pp = popen(ps, "r")) != NULL)
	{
		//DEBUG_LOG("%s open %s.\n", __func__, ps);
		while(fgets(line, sizeof(line), pp) != NULL)
		{
			//DEBUG_LOG("%s, read out line is %s.\n", __func__, line);
			if(strlen(line) > 1)
				strcat(result, line);
		}
		pclose(pp);
		pp = NULL;
	}
	else
	{
		DEBUG_LOG("popen %s error\n", ps);
	}
}

bool fota_program_symlink()
{
	bool bRes = false;
	char chCmd[32] = {0},
	     chLine[64] = {0};

	if(check_file_exist(FOTA_COMMAND_PATH))
	{
		bRes = true;
		DEBUG_LOG("%s, %s has exist.\n", __func__, FOTA_COMMAND_PATH);
		return bRes;
	}

	sprintf(chCmd, "%s", FOTA_COMMAND_SYMLINK);
	FILE *pF = popen(chCmd, "r");

	if(pF != NULL)
	{
		while(fgets(chLine, sizeof(chLine), pF) != NULL)
		{
			DEBUG_LOG("%s, read out line is %s.\n", __func__, chLine);
		}

		bRes = true;
		pclose(pF);
		pF = NULL;
		DEBUG_LOG("%s execute %s OK.\n", __func__, FOTA_COMMAND_SYMLINK);
	}

	return bRes;
}


bool fota_battery_is_charging()
{
	bool bRes = false;
	char chChargeringStatus[16] = {0},
	     chCharingRef[] = {'C', 'h', 'a', 'r', 'g', 'i', 'n', 'g', '\0'};

	if(read_file(BATTERY_STATUS_PATH, chChargeringStatus, sizeof(chChargeringStatus)) <= 0)
	{
		DEBUG_LOG("charging read %s failed!\n",
				BATTERY_STATUS_PATH);
		return bRes;
	}

	DEBUG_LOG("%s status=%s, chCharingRef=%s\n",
			__func__,
			chChargeringStatus,
			chCharingRef);

	if (0 == strcmp(chChargeringStatus, chCharingRef))
	{
		bRes = true;
	}

	return bRes;
}

bool fota_battery_soc_condition()
{
	bool bRes = false;
	int  nCapacity = 0;

	if(read_file_int(BATTERY_CAPATITY_PATH, &nCapacity) != 0)
	{
		DEBUG_LOG("charging read %s failed!\n",
				BATTERY_CAPATITY_PATH);
		return bRes;
	}

	DEBUG_LOG("%s Capacity=%d\n",
			__func__, nCapacity);

	if (nCapacity >= FOTA_UPGRADE_LIMITED_SOC)
	{
		bRes = true;
	}

	return bRes;
}

bool fota_is_running()
{
	bool bRes = false;
	char buf[64] = {0};

	memset(buf, 0, sizeof(buf));

	fota_local_exe_cmd(FOTA_PROCESS_RUNNING_JUDGE, buf, sizeof(buf));
	DEBUG_LOG("%s, Fota is running: %s.\n", __func__, buf);

	if(strstr(buf, FOTA_COMMAND))
	{
		bRes = true;
	}

	return bRes;
}

bool fota_setup_network_connection()
{
	bool bRet = false;
	FILE *fNetConnectingResultStream = NULL;
	char buf[256] = {0};
	unsigned int unSleepTime = 10;


	// 1.setup network connection
	pid_t pid = fork();
	if (pid == 0)
	{
		DEBUG_LOG("%s, start network connecting thread, pid is 0.\n", __func__);
		system(NETWORK_OPERATE_COMMAND);
	}
	else
	{
		DEBUG_LOG("%s, fota main pid is %d.\n", __func__, pid);
	}

Wait_for_network:
	sleep(unSleepTime);

	if(check_file_exist(NETWORK_CONNECTING_RESULT_PATH))
	{
		DEBUG_LOG("%s read %s.\n", __func__, NETWORK_CONNECTING_RESULT_PATH);
		memset(buf, 0, sizeof(buf));
		fNetConnectingResultStream = fopen(NETWORK_CONNECTING_RESULT_PATH, "r");
		if (NULL != fNetConnectingResultStream)
		{
			fread(buf, 1, sizeof(buf), fNetConnectingResultStream);
			fclose(fNetConnectingResultStream);

			DEBUG_LOG("%s buf=%s.\n", __func__, buf);

			if (NULL ==  strstr(buf, NETWORK_CONNECTING_RESULT_EVENT))
			{
				DEBUG_LOG("%s can't connecting network. \n", __func__);
				return bRet;
			}
		}
	}
	else
	{
		unSleepTime = 1;
		DEBUG_LOG("%s %s not exist, wait %d second.\n",
				__func__, NETWORK_CONNECTING_RESULT_PATH, unSleepTime);
		goto Wait_for_network;
	}

	// 2. config dns
	memset(buf, 0, sizeof(buf));
	do
	{
		fota_local_exe_cmd(NETWORK_DNS_CONFIG_COMMAND, buf, sizeof(buf));
		DEBUG_LOG("%s, config DNS: %s.\n", __func__, buf);
	}while(NULL == strstr(buf, NETWORK_DNS_CONFIG_RESULT_STRING));

	//3. verify ip address
	memset(buf, 0, sizeof(buf));
	fota_local_exe_cmd(NETWORK_DNS_CONFIG_VERIFY_COMMAND, buf, sizeof(buf));
	DEBUG_LOG("%s, Verify IP address: %s.\n", __func__, buf);
	if(strlen(buf))
	{
		DEBUG_LOG("%s Connect network ok :):):)\n", __func__);

		// 4. record connection network flag
		g_pFotaDeviceInfo->bNetWorkConnected = true;
		bRet = true;
	}

	return bRet;
}

//static void * fota_handle_thread(void *s)
void fota_handle_thread()
{
	while(1)
	{
		if((NULL != g_pFotaDeviceInfo) && //fota deviceInfo has been initialized
				fota_battery_is_charging())//device connected usb line and is charging.
		{
			if (strlen(g_pFotaDeviceInfo->chDevicesSerialNumber) < 15)
			{
				fota_init_devices_information();
			}
			else
			{
				DEBUG_LOG("%s, SN:%s ok. VER:%s ok.\n", __func__, g_pFotaDeviceInfo->chDevicesSerialNumber,
																g_pFotaDeviceInfo->chDevicesVersion);
			}

			DEBUG_LOG("%s network state: %d.\n",
					__func__, g_pFotaDeviceInfo->bNetWorkConnected);

			// case 1. report fota result to network
			if((!g_pFotaDeviceInfo->bDevicesHasReportedResult) && //devices have reported fota result to server?
					check_file_exist(FOTA_UPGRADE_RESULT_PATH))//devices have completed fota upgrade
			{
				//g_pFotaDeviceInfo->bDevicesIsUpgrade = true;

				// a. setup network
				if(!g_pFotaDeviceInfo->bNetWorkConnected)
				{
					if(fota_setup_network_connection())
					{
						DEBUG_LOG("%s upload fota result. setup network ok.\n", __func__);

						continue;
					}
				}
				else
				{
					// b. delted flag files
					g_pFotaDeviceInfo->bDevicesHasReportedResult = true;
					clear_file(NETWORK_CONNECTING_RESULT_PATH);

					// c. report fota result to server
					DEBUG_LOG("%s, Have connected network, should report fota result to server. :):):)\n", __func__);
					execlp(FOTA_COMMAND_PATH,
							FOTA_COMMAND,
							NULL);
				}
			}

			// case 2. start normal fota program
			if ((!g_pFotaDeviceInfo->bDevicesIsUpgrading) && //devices don't complete fota operation
					fota_battery_soc_condition())//battery condition
			{
				// a. Setup network connection
				if(!g_pFotaDeviceInfo->bNetWorkConnected)
				{
					if(fota_setup_network_connection())
					{
						DEBUG_LOG("%s Fota upgrade. setup network ok.\n", __func__);
						continue;
					}
				}
				else
				{
					// b. modify flag
					g_pFotaDeviceInfo->bDevicesIsUpgrading = true;
					clear_file(NETWORK_CONNECTING_RESULT_PATH);

					// c. start fota upgrade program
					DEBUG_LOG("%s entry Fota upgrade program, SN:%s :):):).\n",
							__func__, g_pFotaDeviceInfo->chDevicesSerialNumber);

					/*execlp("/sbin/ljtao",
					  "ljtao",
					  NULL);
					//system("/sbin/ljtao");*/
					execlp(FOTA_COMMAND_PATH,
							FOTA_COMMAND,
							g_pFotaDeviceInfo->chDevicesSerialNumber,
							g_pFotaDeviceInfo->chDevicesBrand,
							g_pFotaDeviceInfo->chDevicesModel,
							g_pFotaDeviceInfo->chDevicesVersion,
							NULL);
				}
			} 
		}
		sleep(FOTA_POLL_INTERVAL_TIME);
	}
}


/*void healthd_prepare_for_fota()
  {
// 1. init devices info for fota
if(fota_init_devices_information())
{
// 2. create a thread for download fota-image and wait for download OK.
pthread_t pid_fota_thread;
int ret = pthread_create(&pid_fota_thread, NULL, fota_handle_thread, NULL);
if(ret < 0)
{
DEBUG_LOG("Can't create fota pthread: %s.\n", strerror(errno));
}
else
{
DEBUG_LOG("%s fota thread has created, pid is %lu. \n", __func__, pid_fota_thread);
}
pthread_join(pid_fota_thread, NULL);
}
}*/

int healthd_send_command_to_mcu(struct charger *charger, bool bSelfTest)
{
	uint8_t nChargeData[20] = {0xFD, 0x00, 0x14, 0x00,
		0xB0, 0x00, 0x01, 0x50,
		0x80, 0x00, 0x01, 0xA2,
		0x81, 0x00, 0x01, 0x00,//index-15 is red or yellow blink
		0x82, 0x00, 0x01, 0x00},//index-19 is soc
		nSelfTestData[12] = {0xFD,  0x00, 0x0C, 0x00,
			0xB0, 0x00, 0x01, 0x50,
			0x80, 0x00, 0x01, 0x81},    
		*pData = NULL, nLen = 0;
	int ret = -1;

	//serialization command to LTUD
	if(!bSelfTest)
	{
		nChargeData[15] = charger->cls;
		nChargeData[19] = charger->capacity;

		pData = nChargeData;
		nLen = sizeof(nChargeData)/sizeof(uint8_t);

		DEBUG_LOG("%s, cls=%d, soc=%d, nLen=%d.\n", __func__, nChargeData[15], nChargeData[19], nLen);
	}
	else//selfTest Command
	{
		pData = nSelfTestData;
		nLen = sizeof(nSelfTestData)/sizeof(uint8_t);

		DEBUG_LOG("%s, selfTest nlen=%d.\n", __func__, nLen);
	}


	ret = write(g_ltud_sock, (unsigned char*)pData, nLen);

	if(!ret)
	{
		DEBUG_LOG("Write data to mcu error.\n");
	}
	else
	{
		DEBUG_LOG("Write data to mcu ok. ret=%d\n", ret);
	}

}

void healthd_mode_show_led(struct charger *charger)
{
	/*
charging: blink-yellow
charge complete: green led

charging not complete, plug from dock:
battery-capacity > 95%   : power off
battery-capacity < 30%   : blink red
battery-capacity 30%-95% : blink yellow
	 */

	bool bPowerOff = false;
	ChargerLEDStatus cls = NONELED;

	if (charger->charger_connected) //devices is charging
	{
		if (100 == charger->capacity)
		{
			//power off
			//bPowerOff = true;
			cls = GREEN;
			DEBUG_LOG("charge ok.light green led.\n");
		}
		else
		{
			//blink yellow led
			cls = YELLOW;
			DEBUG_LOG("charging. blink yellow led.\n");
		}
	}
	else if((!charger->charger_connected) &&
			(charger->capacity < 100) &&
			(charger->charger_not_completed))//charging but not complete and plug-out from dock
	{
		if (charger->capacity < 30)
		{
			//nCurrentChargeLevel = ChargeLevelLow;
			//blink red
			cls = RED;
			DEBUG_LOG("charge not complete. blink red led.\n");
		}
		else if((charger->capacity >= 30) && (charger->capacity < 95))
		{
			//blink yellow
			cls = YELLOW;
			DEBUG_LOG("charge not complete. blink yellow led.\n");
		}
		else
		{
			//power off devices
			bPowerOff = true;
			DEBUG_LOG("charge not complete, but more than 95%. power off.\n");

		}
	}  
	else if((!charger->charger_connected) && (100 == charger->capacity))
	{
			//power off devices
			bPowerOff = true;
			DEBUG_LOG("charge complete,now we plug-out from dock. power off.\n");
	}

	if (bPowerOff)
	{
		if(fota_is_running())
		{
			DEBUG_LOG("%s, devices is fota, we can't shutdown devices.\n", __func__);
			return ;
		}

		//we shou power off devices
		pid_t pid = fork();
		if (pid == 0)
		{
			DEBUG_LOG("%s, devices not in fota,  %s.\n", __func__, SHUTDOWN_COMMAND);
			//execl(SHUTDOWN_PATH, SHUTDOWN_COMMAND, NULL);
			system(SHUTDOWN_COMMAND);
		}
		//remove data
		//system(RM_COMMAND);
	}

	DEBUG_LOG("ledStatus=%d, charger->cls=%d, charger->bPowerKeyPressed=%d, soc=%d.\n",
			cls, charger->cls, charger->bPowerKeyPressed, charger->capacity);

	if (charger->bPowerKeyPressed)//user press power key we need start self-test,so we don't need 
	{
		charger->bPowerKeyPressed = false;
		return ;
	}

	if (cls != charger->cls)
	{
		charger->cls = cls;


	}
		//send command to mcu
		healthd_send_command_to_mcu(charger, false);

}

static void handle_power_supply_state(int64_t now)
{
	static int old_soc = 0;
	int soc, ret;
	//int online_check = 0;

	/* if (!charger->have_battery_state)
	   return;

	   soc = get_battery_capacity();
	   if(soc == BATTERY_FULL_THRESH) {
	   close_green_led();
	   }else{
	   open_green_led();
	   }*/

	//get usb status
	ret = read_file_int(USB_ONLINE_PATH,&g_online_check);
	if( ret != 0 )
		return ;

	if(1 == g_online_check)
	{
		/*if (USB_CABLE_PLUG_IN_COUNT_TIMES_DURING == ++g_nUsbCablePlugInCount)
		  {		    
		  healthd_entry_fota();
		  }
		  else if (g_nUsbCablePlugInCount > 10)
		  {
		  g_nUsbCablePlugInCount = USB_CABLE_PLUG_IN_COUNT_TIMES_DURING + 1;
		  }*/

		DEBUG_LOG("%s, g_online_check = %d, g_nUsbCablePlugInCount=%d.\n",
				__func__, g_online_check, g_nUsbCablePlugInCount);
	}
	else if (0 == g_online_check)
	{
		//g_charger_state.charger_not_completed = true;
	}
	//DEBUG_LOG("%s , g_online_check=%d.\n", __func__, g_online_check);

}

void healthd_mode_charger_heartbeat()
{
	struct charger *charger = &g_charger_state;
	int64_t now = curr_time_ms();
	int ret;

	//DEBUG_LOG("%s now=%ld.\n", __func__, now);
	//handle_input_state(charger, now);
	handle_power_supply_state(now);
}

void healthd_mode_charger_battery_update(
		struct BatteryProperties *props)
{
	struct charger *chargerTemp = &g_charger_state;

	chargerTemp->charger_connected =
		props->chargerAcOnline || props->chargerUsbOnline ||
		props->chargerWirelessOnline;
/*
	chargerTemp->capacity = props->batteryLevel;
	if(!chargerTemp->capacity)
	{
		read_file_int(BATTERY_CAPATITY_PATH,&chargerTemp->capacity);
	}
*/
	read_file_int(BATTERY_CAPATITY_PATH,&chargerTemp->capacity);

	DEBUG_LOG("%s battery soc %d.\n", __func__, chargerTemp->capacity);

	//updata charge cable status
	if ((chargerTemp->charger_connected) && 
			(chargerTemp->capacity < 100))//It's charging, so we can detected charge-line
	{
		chargerTemp->chargeLineHasbeenDetected = true;
	}

	//add for debug begin
#ifdef DEBUG_USB_CABLE
	if ((chargerTemp->charger_connected) && 
			(chargerTemp->capacity == 88)) //just for test
	{
		g_nJustforDebugLed = 1;
		chargerTemp->charger_connected = false;
	}
	else if(chargerTemp->capacity == 66)
	{
		g_nJustforDebugLed = 0;
	}

	if(g_nJustforDebugLed)
	{
		chargerTemp->charger_connected = false;
	}
#endif
	//add for debug end

	if(((!chargerTemp->charger_connected) && 
				(chargerTemp->capacity < 100) &&
				(chargerTemp->chargeLineHasbeenDetected)) || (g_nJustforDebugLed == 1) ) //charge not completed and plug-out charge-line
	{
		chargerTemp->chargeLineHasbeenDetected = false;
		chargerTemp->charger_not_completed = true;
	}

	//show led indetification
	healthd_mode_show_led(chargerTemp);

	/*if (!charger->have_battery_state) {
	  charger->have_battery_state = true;
	  charger->next_screen_transition = curr_time_ms() - 1;
	  reset_animation(charger->batt_anim);
	  kick_animation(charger->batt_anim);
	  }
	  batt_prop = props;*/	
}

int healthd_mode_charger_preparetowait(void)
{
	int64_t timeout = -1;
#if 0
	struct charger *chargerTemp = &g_charger_state;
	int64_t now = curr_time_ms();
	int64_t next_event = INT64_MAX;


	DEBUG_LOG("%s next key: %" PRId64 " next pwr: %" PRId64 "\n", now,
			__func__, chargerTemp->next_key_check, chargerTemp->next_pwr_check);

	/*if (charger->next_screen_transition != -1)
	  next_event = charger->next_screen_transition;
	  if (charger->next_key_check != -1 && charger->next_key_check < next_event)
	  next_event = charger->next_key_check;
	  if (charger->next_pwr_check != -1 && charger->next_pwr_check < next_event)
	  next_event = charger->next_pwr_check;*/

	if (next_event != INT64_MAX)
		timeout = max(0, next_event - now);
	else
		timeout = -1;

#endif

	//DEBUG_LOG("%s timeout=%ld.\n", __func__, timeout);

	return (int)timeout;
}


static int set_key_callback(int code, int value, void *data)
{
	struct charger *chargerTemp = (struct charger *)data;
	int64_t now = curr_time_ms();
	int down = !!value;

	if (code > KEY_MAX)
		return -1;

	/* ignore events that don't modify our state */
	if (chargerTemp->keys[code].down == down)
		return 0;

	/* only record the down even timestamp, as the amount
	 * of time the key spent not being pressed is not useful */
	if (down)
		chargerTemp->keys[code].timestamp = now;
	chargerTemp->keys[code].down = down;
	chargerTemp->keys[code].pending = true;
	if (down) {
		DEBUG_LOG("%s, [%" PRId64 "] key[%d] down\n", __func__, now, code);
	} else {
		int64_t duration = now - chargerTemp->keys[code].timestamp;
		int64_t secs = duration / 1000;
		int64_t msecs = duration - secs * 1000;
		DEBUG_LOG("%s, [%" PRId64 "] key[%d] up (was down for %" PRId64 ".%" PRId64 "sec)\n",
				__func__, now, code, secs, msecs);

		//we should do self-test if Power Key press duration 150ms
		if ((msecs > 150) && (code == KEY_POWER))
		{
			DEBUG_LOG("Power key pressed. We should selftest devices.");
			chargerTemp->bPowerKeyPressed = true;
			healthd_send_command_to_mcu(chargerTemp, true);
		}
	}

	return 0;
}

static void update_input_state(struct charger *charger,
		struct input_event *ev)
{
	if (ev->type != EV_KEY)
	{
		//DEBUG_LOG("%s. key_type=%d.\n", __func__, ev->type);
		return;
	}
	set_key_callback(ev->code, ev->value, charger);
}

static int input_callback(int fd, unsigned int epevents, void *data)
{
	struct charger *charger = (struct charger *)data;
	struct input_event ev;
	int ret;

	ret = ev_get_input(fd, epevents, &ev);
	if (ret)
		return -1;
	update_input_state(charger, &ev);
	return 0;
}

static void charger_event_handler(uint32_t /*epevents*/)
{
	int ret;

	ret = ev_wait(-1);
	if (!ret)
		ev_dispatch();
}

int healthd_register_event(int fd, void (*handler)(uint32_t)) {
	struct epoll_event ev;

	ev.events = EPOLLIN | EPOLLWAKEUP;
	ev.data.ptr = (void *)handler;
	if (epoll_ctl(g_epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
		DEBUG_LOG("epoll_ctl failed; errno=%d\n", errno);
		return -1;
	}

	g_eventct++;

	DEBUG_LOG("%s ok. fd=%d, g_eventct=%d.\n", __func__, fd, g_eventct);
	return 0;
}


void healthd_mode_charger_init(struct healthd_config* config)
{
	int ret;
	int charging_enabled = 1;
	struct charger *chargerTemp = &g_charger_state;
	int i;
	int epollfd = -1;
	int nReadCharge = 0;
	//dump_last_kmsg();

	//1. get battery charge status   
	for(nReadCharge=0; nReadCharge<6; nReadCharge++)
	{
		ret = read_file_int(CHARGING_ENABLED_PATH, &charging_enabled);
		if(ret!=0)
		{
			DEBUG_LOG("charging read %s failed!ignore go on charger!\n",
					CHARGING_ENABLED_PATH);
			break;
		}

		if(charging_enabled )
		{
			DEBUG_LOG("%s, charging enabled\n", __func__);
			break;
		}
		else if(nReadCharge==5){
			//android_reboot(ANDROID_RB_POWEROFF, 0, 0);
			DEBUG_LOG("charging read 5 times failed! power off phone\n");
		}
		DEBUG_LOG("charging read %d times failed! try again\n", nReadCharge);
		usleep(100000);	
	}

	ret = ev_init(input_callback, chargerTemp);
	if (!ret) {
		epollfd = ev_get_epollfd();
		healthd_register_event(epollfd, charger_event_handler);
		DEBUG_LOG("%s register key event.\n", __func__);
	}

#if 0
	ev_sync_key_state(set_key_callback, chargerTemp);

	chargerTemp->next_key_check = -1;
	chargerTemp->next_pwr_check = -1;
	//healthd_config = config;
#endif
}

static void periodic_chores() {
	DEBUG_LOG("%s entry.\n", __func__);
	healthd_battery_update();
}

static void uevent_event(uint32_t /*epevents*/) {
	char msg[UEVENT_MSG_LEN+2];
	char *cp;
	int n;

	n = uevent_kernel_multicast_recv(g_uevent_fd, msg, UEVENT_MSG_LEN);
	if (n <= 0)
		return;
	if (n >= UEVENT_MSG_LEN)   /* overflow -- discard */
		return;

	msg[n] = '\0';
	msg[n+1] = '\0';
	cp = msg;

	//DEBUG_LOG("%s ok. n=%d.\n", __func__, n);

	while (*cp) {
		if (!strcmp(cp, "SUBSYSTEM=" POWER_SUPPLY_SUBSYSTEM)) {
			healthd_battery_update();
			break;
		}

		/* advance to after the next \0 */
		while (*cp++)
			;
	}
}

static void uevent_init(void) {
	g_uevent_fd = uevent_open_socket(64*1024, true);

	if (g_uevent_fd < 0) {
		DEBUG_LOG("uevent_init: uevent_open_socket failed\n");
		return;
	}

	fcntl(g_uevent_fd, F_SETFL, O_NONBLOCK);
	if (healthd_register_event(g_uevent_fd, uevent_event))
		DEBUG_LOG("register for uevent events failed\n");

	DEBUG_LOG("%s ok. g_uevent_fd=%d.\n", __func__, g_uevent_fd);
}

static void wakealarm_event(uint32_t /*epevents*/) {
	unsigned long long wakeups;

	if (read(g_wakealarm_fd, &wakeups, sizeof(wakeups)) == -1) {
		DEBUG_LOG("wakealarm_event: read wakealarm fd failed\n");
		return;
	}	

	DEBUG_LOG("%s ok. wakealarm_fd=%d.\n", __func__, g_wakealarm_fd);

	periodic_chores();
}

static void wakealarm_set_interval(int interval) {
	struct itimerspec itval;

	DEBUG_LOG("%s, g_wakealarm_fd=%d, wakealarm_wake_interval=%d, interval=%d.\n",
			__func__, g_wakealarm_fd, wakealarm_wake_interval, interval);
	if (g_wakealarm_fd == -1)
		return;

	wakealarm_wake_interval = interval;

	if (interval == -1)
		interval = 0;

	itval.it_interval.tv_sec = interval;
	itval.it_interval.tv_nsec = 0;
	itval.it_value.tv_sec = interval;
	itval.it_value.tv_nsec = 0;

	/*DEBUG_LOG("%s, set timers.\n", __func__);

	  if (timerfd_settime(g_wakealarm_fd, 0, &itval, NULL) == -1)
	  DEBUG_LOG("wakealarm_set_interval: timerfd_settime failed\n");
	  else
	  DEBUG_LOG("%s OK, interval=%d", wakealarm_wake_interval);*/

	DEBUG_LOG("%s, ok.\n", __func__);
}

static void wakealarm_init(void) {
	g_wakealarm_fd = timerfd_create(CLOCK_BOOTTIME_ALARM, TFD_NONBLOCK);
	if (g_wakealarm_fd == -1) {
		DEBUG_LOG("wakealarm_init: timerfd_create failed\n");
		return;
	}

	if (healthd_register_event(g_wakealarm_fd, wakealarm_event))
		DEBUG_LOG("Registration of wakealarm event failed\n");

	wakealarm_set_interval(wakealarm_wake_interval);

	//DEBUG_LOG("%s ok.\n", __func__);
}

int mcu_communication_init()
{
	struct sockaddr_un server;
	socklen_t alen = 0;
	int ret = 0;	

	g_ltud_sock = socket(PF_UNIX, SOCK_STREAM, 0);
	if(g_ltud_sock < 0) {
		DEBUG_LOG("Failed to open socket '%s': %s", LTUD_SOCKET, strerror(errno));
		return -1;
	}
	/** Initialize address structure */
	memset(&server, 0, sizeof(struct sockaddr_un));

	/** Set address family to unix domain sockets */
	server.sun_family = AF_UNIX;

	/** Set address to the requested pathname */
	snprintf(server.sun_path, sizeof(server.sun_path), "%s", LTUD_SOCKET);

	/** Get length of pathname */
	alen = strlen(server.sun_path) + sizeof(server.sun_family);

	while(1) {
		ret = connect(g_ltud_sock, (struct sockaddr *) &server, alen);
		if(ret == 0)
		{
			DEBUG_LOG("EXIT  path=%s. ", LTUD_SOCKET);
			break;
		}

		sleep(1);
	}
	DEBUG_LOG("Connected to server socket '%s': sock=%d.\n", LTUD_SOCKET, g_ltud_sock);
}
static int healthd_init() {
	g_epollfd = epoll_create(MAX_EPOLL_EVENTS);
	if (g_epollfd == -1) {
		DEBUG_LOG("epoll_create failed; errno=%d\n",
				errno);
		return -1;
	}	

	ghealthd_mode_ops->init(&g_healthd_config);
	uevent_init();
	wakealarm_init();
	mcu_communication_init();

	gBatteryMonitor = new BatteryMonitor();
	gBatteryMonitor->init(&g_healthd_config);

	//DEBUG_LOG("%s ok.\n", __func__);

	return 0;
}


void healthd_battery_update(void) {
	// Fast wake interval when on charger (watch for overheat);
	// slow wake interval when on battery (watch for drained battery).

	//DEBUG_LOG("%s, entry.\n", __func__);
	int new_wake_interval = gBatteryMonitor->update() ?
		g_healthd_config.periodic_chores_interval_fast :
		g_healthd_config.periodic_chores_interval_slow;

	if (new_wake_interval != wakealarm_wake_interval)
		wakealarm_set_interval(new_wake_interval);

	// During awake periods poll at fast rate.  If wake alarm is set at fast
	// rate then just use the alarm; if wake alarm is set at slow rate then
	// poll at fast rate while awake and let alarm wake up at slow rate when
	// asleep.

	if (g_healthd_config.periodic_chores_interval_fast == -1)
		g_awake_poll_interval = -1;
	else
		g_awake_poll_interval =
			new_wake_interval == g_healthd_config.periodic_chores_interval_fast ?
			-1 : g_healthd_config.periodic_chores_interval_fast * 1000;

	//DEBUG_LOG("%s, new_wake_interval=%d, wakealarm_wake_interval=%d, g_awake_poll_interval=%d.\n",
	//	__func__, new_wake_interval, wakealarm_wake_interval, g_awake_poll_interval);
}


static void healthd_mainloop(void) {
	while (1) {
		struct epoll_event events[g_eventct];
		int nevents;
		int timeout = g_awake_poll_interval;
		int mode_timeout;

		mode_timeout = ghealthd_mode_ops->preparetowait();
		if (timeout < 0 || (mode_timeout > 0 && mode_timeout < timeout))
			timeout = mode_timeout;
		nevents = epoll_wait(g_epollfd, events, g_eventct, timeout);

		if (nevents == -1) {
			if (errno == EINTR)
				continue;
			DEBUG_LOG("%s, healthd_mainloop: epoll_wait failed\n", __func__);
			break;
		}

		for (int n = 0; n < nevents; ++n) {
			if(!g_online_check)  //avoid power off block in poll wait event
				break;

			if (events[n].data.ptr)
				(*(void (*)(int))events[n].data.ptr)(events[n].events);
		}

		if (!nevents)
			periodic_chores();

		//DEBUG_LOG("%s, nevents=%d.\n", __func__, nevents);
		ghealthd_mode_ops->heartbeat();
	}

	return;
}


int main(int argc, char **argv) {
	int ret;

	if (argc >= 2)
	{
		g_chDebugMask = (unsigned char)atoi(argv[1]);
	}

	ghealthd_mode_ops = &charger_ops;

	ret = healthd_init();
	if (ret) {
		DEBUG_LOG("Initialization failed, exiting\n");
		exit(2);
	}

	periodic_chores();
	ghealthd_mode_ops->heartbeat();

	pid_t pid = fork();
	if (pid == 0)//create a new thread for fota
	{
		DEBUG_LOG("%s, start a new thread for FOTA, pid is 0.\n", __func__);
		// 1. init devices info for fota
		if(fota_init_devices_information())
		{
			// 2. rename fota program
			if(fota_program_symlink())
			{
				// 3. fota handle thread.
				fota_handle_thread();
			}
			else
			{
				DEBUG_LOG("%s, Rename fota progrom eror.\n", __func__);
			}
		}
		else
		{
			DEBUG_LOG("%s, Init device information eror.\n", __func__);
		}
	}
	else//main thread for battery charge monitor
	{
		DEBUG_LOG("%s, main process pid is %d.\n", __func__, pid);
		healthd_mainloop();
		DEBUG_LOG("Main loop terminated, exiting\n");
	}

	return 0;
}
