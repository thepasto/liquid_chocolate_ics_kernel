/*
* Customer code to add GPIO control during WLAN start/stop
* Copyright (C) 1999-2009, Broadcom Corporation
* 
*         Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2 (the "GPL"),
* available at http://www.broadcom.com/licenses/GPLv2.php, with the
* following added to such license:
* 
*      As a special exception, the copyright holders of this software give you
* permission to link this software with independent modules, and to copy and
* distribute the resulting executable under terms of your choice, provided that
* you also meet, for each linked independent module, the terms and conditions of
* the license of that module.  An independent module is a module which is not
* derived from this software.  The special exception does not apply to any
* modifications of the software.
* 
*      Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*
* $Id: dhd_custom_gpio.c,v 1.1.2.7 2009/11/06 11:18:43 Exp $
*/

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>

#include <dngl_stats.h>
#include <dhd.h>

#include <wlioctl.h>
#include <wl_iw.h>

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#include <linux/gpio.h>
#include <mach/gpio.h>
#if defined(CONFIG_MMC_WIFI) || defined(CONFIG_MMC_WIFI_MODULE)
#define WL_PWR_EN 109
#define WL_RST 147
#endif

static int wifi_power(int on)
{
    int bt_on = 0;
    pr_debug("%s\n", __func__);

    //In order to follow wifi power sequence, we have to detect bt power status

    if (on) {
        bt_on=gpio_get_value(106);
        if(!bt_on){
            //if WLAN on and BT off
            gpio_set_value(WL_PWR_EN, 1); /* WL_PWR_EN */
            gpio_set_value(106, 1); /* BT_PWR_EN */
            msleep(100);
            gpio_set_value(WL_RST, 1); /* WL_RST */
            gpio_set_value(31, 1); /* BT_RST */
            msleep(100);//at last 100 ms
            gpio_set_value(31, 0); /* BT_RST */
            gpio_set_value(106, 0); /* BT_PWR_EN */
        } else {
            //if WLAN on and BT on
            gpio_set_value(WL_PWR_EN, 1); /* WL_PWR_EN */
            msleep(100);
            gpio_set_value(WL_RST, 1); /* WL_RST */
        }

        pr_info(KERN_INFO"Wifi Power ON\n");

    } else {
        gpio_set_value(WL_PWR_EN, 0); /* WL_PWR_EN */
        msleep(100);
        gpio_set_value(WL_RST, 0); /* WL_RST */

        pr_info("Wifi Power OFF\n");
    }
    return 0;
}

#ifdef CUSTOMER_HW
extern  void bcm_wlan_power_off(int);
extern  void bcm_wlan_power_on(int);
#endif /* CUSTOMER_HW */

#if defined(OOB_INTR_ONLY)

#ifdef CUSTOMER_HW3
#include <mach/gpio.h>
#endif

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1; /* GG 19 */

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

int dhd_customer_oob_irq_map(void)
{
int  host_oob_irq = 0;
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#if defined CUSTOMER_HW
	host_oob_irq = MSM_GPIO_TO_INT(dhd_oob_gpio_num);
#elif defined CUSTOMER_HW3
	gpio_request(dhd_oob_gpio_num, "oob irq");
	host_oob_irq = gpio_to_irq(dhd_oob_gpio_num);
	gpio_direction_input(dhd_oob_gpio_num);
#endif

	return (host_oob_irq);
}
#endif /* defined(OOB_INTR_ONLY) */

/* Customer function to control hw specific wlan gpios */
void
dhd_customer_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
		case WLAN_RESET_OFF:
			WL_TRACE(("%s: call customer specific GPIO to insert WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(2);
#endif /* CUSTOMER_HW */
			wifi_power(0);
			WL_ERROR(("=========== WLAN placed in RESET ========\n"));
		break;

		case WLAN_RESET_ON:
			WL_TRACE(("%s: callc customer specific GPIO to remove WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(2);
#endif /* CUSTOMER_HW */
			wifi_power(1);
			WL_ERROR(("=========== WLAN going back to live  ========\n"));
		break;

		case WLAN_POWER_OFF:
			WL_TRACE(("%s: call customer specific GPIO to turn off WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(1);
#endif /* CUSTOMER_HW */
		break;

		case WLAN_POWER_ON:
			WL_TRACE(("%s: call customer specific GPIO to turn on WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(1);
#endif /* CUSTOMER_HW */
			/* Lets customer power to get stable */
			OSL_DELAY(500);
		break;
	}
}
