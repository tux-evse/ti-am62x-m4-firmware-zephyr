/*
 * Copyright (c) 2025, VALEO
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* HISTORY:
---------------------------------------------------------------------------------
| Version |    Date    | Name |                     Comment                     !
|---------|------------|------|-------------------------------------------------|
|   0.1   | 04/02/2025 |  FLE | Initial version: Rpmsg communication between    |
|         |            |      | Linux [core A53] & M4F firmware [core M4F]      |
|         |            |      | (use Mailbox driver & Integration of Protobuf   |
|         |            |      | from freeRTOS).                                 |
|---------|------------|------|-------------------------------------------------|
|   0.2   | 05/02/2025 |  FLE | Adding the LED0 HEARTBEAT (Freq = 0.5 Hz).      |
|         |            |      | Adding Relay control handle.                    |
|---------|------------|------|-------------------------------------------------|
|   0.3   | 10/02/2025 |   AA | Use IPM API's instead of MBOX                   |
|         |            |      | Config IPM to MBOX passthrough driver           |
|---------|------------|------|-------------------------------------------------|

*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/ipm.h>
#include <zephyr/drivers/gpio.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ti_am62x_m4_firmware, LOG_LEVEL_DBG);

//FLE adding:
#include "pb_codec/nanopb/pb_decode.h"
#include "pb_codec/high_to_low.pb.h"
//------------

#define SHM_DEVICE_NAME	"shm"

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_LinuxMsg_stack, APP_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_LinuxMsg_data;

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static struct fw_resource_table *rsc_table;
static struct rpmsg_device *rpdev;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_linuxmsg_sem, 0, 1);

static struct rpmsg_endpoint linuxmsg_ept;

/* Protobuf buffers */
static uint8_t msg_buffer[HighToLow_size];
static struct rpmsg_rcv_msg linuxmsg_msg = {.data = (void*)&msg_buffer};
static HighToLow rx_linuxmsg_msg;

//LED0 HEARTBEAT
#define LED0_HEARTBEAT_TIMING (1000) //One second
#define LED0_NODE DT_ALIAS(led0) /* The devicetree node identifier */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

//RELAY CONTROL
#define RELAY_CTL0_NODE DT_ALIAS(relayctl0) /* The devicetree node identifier */
static const struct gpio_dt_spec relayctl = GPIO_DT_SPEC_GET(RELAY_CTL0_NODE, gpios);

//FLE: Exemple to handle incoming linux message--------------------------------------------

/* Field tags (for use in manual encoding/decoding) */
#define SetPWM_state_tag                         1
#define SetPWM_duty_cycle_tag                    2
#define SetSLAC_state_tag                        1
#define HighToLow_set_pwm_tag                    1
#define HighToLow_allow_power_on_tag             2
#define HighToLow_enable_tag                     3
#define HighToLow_disable_tag                    4
#define HighToLow_heartbeat_tag                  5
#define HighToLow_set_slac_tag                   6

/* Handle incoming Linux message sent by core A53  */
void handle_incoming_message(const HighToLow* in) {
    if (in->which_message == HighToLow_set_pwm_tag){
        SetPWM set_pwm = in->message.set_pwm;


        switch (set_pwm.state) {
        case PWMState_F:
            LOG_INF("MODE PWMState_F");
            break;
        case PWMState_OFF:
            LOG_INF("MODE PWMState_OFF");
            break;
        case PWMState_ON:
            LOG_INF("MODE PWMState_ON [pwm msg received %f]", (double)set_pwm.duty_cycle);
            break;
        default:
            // NOT ALLOWED
			LOG_INF("MODE NOT ALLOWED !!");
            break;

        LOG_INF("PWM STATE : %d, / PWM DC : %f",set_pwm.state,(double)set_pwm.duty_cycle);
        }
    } else if (in->which_message == HighToLow_allow_power_on_tag) {
        bool intEnable = in->message.allow_power_on;
		int ret;

        LOG_INF("power_on msg received, enable variable = %d",intEnable);
		//Handle the relay control: GPIO0 Pin 14 (MCU_GPIO0_14 / MCU_MCAN0_RX)
		ret = gpio_pin_set_dt(&relayctl, (int)intEnable );
		if (ret != 0)
		{
			LOG_ERR("GPIO gpio_pin_set_dt relayctl failed");
		}
    } else if (in->which_message == HighToLow_enable_tag) {
        LOG_INF("Received a enable Tag from CPU");
    } else if (in->which_message == HighToLow_disable_tag) {
        LOG_INF("Received a disable Tag from CPU");
    } else if (in->which_message == HighToLow_set_slac_tag) {
        LOG_INF("Received a SLAC Status from the CPU");
        SetSLAC set_slac = in->message.set_slac;
        switch (set_slac.state) {
        case SLACState_RUN :
            LOG_INF("SLAC STATE = SLACState_RUN");
            break;
        case SLACState_OK :
            LOG_INF("SLAC STATE = SLACState_OK");
            break;
        case SLACState_NOK :
            LOG_INF("SLAC STATE = SLACState_NOK");
            break;
        default:
            LOG_INF("unknown SLACstate message");
            break;
        }//END SWITCH
    } else if (in->which_message == HighToLow_heartbeat_tag) {
        LOG_INF("Received a heartbeat from the CPU");
    }
    else {
        LOG_INF("Unknown coming message: %d",in->which_message);
    }
}
//-----------------------------------------------------------------------------------------

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("%s: msg received from mb %d", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_linuxmsg_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	if (len > (size_t)HighToLow_size)
	{
		LOG_ERR("[RPMSG] Linux Message received truncated due to insufficient user buffer size : %d !!!", len);
		len = (size_t)HighToLow_size;
	}

	memcpy(linuxmsg_msg.data, data, len);
	linuxmsg_msg.len = len;
	k_sem_give(&data_linuxmsg_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	int status = k_sem_take(&data_sem, K_FOREVER);

	if (status == 0) {
		rproc_virtio_notified(rvdev.vdev, VRING1_ID);
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	LOG_DBG("%s: msg received", __func__);
	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}

int platform_init(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d", status);
		return -1;
	}
	LOG_INF("metal_init: OK");

	/* declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	/* declare resource table region */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	metal_io_init(rsc_io, rsc_table,
		      &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -1;
	}
	LOG_INF("IPM device is OK ready");

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed");
		return -1;
	}
	LOG_INF("ipm_set_enabled OK");

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}
	LOG_INF("Ok to create vdev");

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}
	LOG_INF("Ok to init vring 0");

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}
	LOG_INF("Ok to init vring 1");

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}
	LOG_INF("Ok rpmsg_init_vdev");

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder started");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform");
		ret = -1;
		goto task_end;
	}
	LOG_INF("OK to initialize platform");

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device");
		ret = -1;
		goto task_end;
	}
	LOG_INF("OK to create rpmsg virtio device");

#ifdef CONFIG_SHELL_BACKEND_RPMSG
	(void)shell_backend_rpmsg_init_transport(rpdev);
#endif

	/* start the rpmsg clients */
	k_sem_give(&data_linuxmsg_sem);
	

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP ended");
}


void app_rpmsg_linuxmsg(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int ret = 0, k_sem_status;
	uint64_t last_toggleled0_ts, last_heartbeat_ts, current_ts;
	pb_istream_t istream;

	k_sem_take(&data_linuxmsg_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux message responder started");

	ret = rpmsg_create_ept(&linuxmsg_ept, rpdev, "rpmsg_chrdev",
			       14,RPMSG_ADDR_ANY, //RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_linuxmsg_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux message client] Could not create endpoint: %d", ret);
		goto task_end;
	}
	LOG_INF("[Linux message client] OK to create endpoint");

	last_toggleled0_ts = last_heartbeat_ts = sys_clock_tick_get();
	while (1) {
		//k_sem_take(&data_linuxmsg_sem,  K_FOREVER);
		k_sem_status = k_sem_take(&data_linuxmsg_sem,  K_MSEC(500));
		if (k_sem_status == 0)
		{
			LOG_INF("[Linux message client] ==> Incoming msg (size: %.d)", linuxmsg_msg.len);

			/* Protobuf decoding: 
			   linuxmsg_msg contains the encoded linux message (received from IPC/mailbox)
			   rx_linuxmsg_msg contains the decoded linux message
			*/
			istream = pb_istream_from_buffer((pb_byte_t *)linuxmsg_msg.data, linuxmsg_msg.len);
			if (true == pb_decode(&istream, HighToLow_fields, &rx_linuxmsg_msg))
			{
				//Process incoming linux message
				handle_incoming_message(&rx_linuxmsg_msg);
			}
			else
			{
				LOG_INF("[Linux message client] Protbuf decoding -  Bad linux message !!");
			}

		}
		else if (k_sem_status == -EAGAIN)
		{
			//LOG_INF("[Linux message client] Semaphore Timeout elapsed");
		}
		else if (k_sem_status == -EBUSY)
		{
			LOG_INF("[Linux message client] Semaphore Busy");
		}
		else
		{
			LOG_INF("[Linux message client] Semaphore ???");
		}


		current_ts = sys_clock_tick_get();


		//LED0 HEARTBEAT
		if (k_ms_to_ticks_floor64((uint64_t)(LED0_HEARTBEAT_TIMING)) < (current_ts - last_toggleled0_ts)) 
		{
			int ret;

			last_toggleled0_ts = current_ts;

			ret = gpio_pin_toggle_dt(&led);
			if (ret < 0) 
			{
				LOG_WRN("GPIO gpio_pin_toggle_dt led failed");
			}
		}

		//Send Heartbeat to linux
        if (k_ms_to_ticks_floor64((uint64_t)(3000)) < (current_ts - last_heartbeat_ts)) 
		{
           last_heartbeat_ts = current_ts;
 
 		 	//LOG_INF("[Linux message client] Send Heartbeat");
 			//rpmsg_send(&linuxmsg_ept, sc_msg.data, sc_msg.len);
        }
	
	}
	rpmsg_destroy_ept(&linuxmsg_ept);

task_end:
	LOG_INF("OpenAMP[remote] Linux message responder ended");
}

//LED0 HEARTBEAT initialization
bool init_led0(void)
{
	int ret;

	LOG_INF("Start led0 init");

	if (!gpio_is_ready_dt(&led)) 
	{
		LOG_WRN("GPIO led is not ready");
		return false;
	}
	else 
	{
		LOG_INF("GPIO led is ready");
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
	 	LOG_WRN("GPIO gpio_pin_configure_dt led failed");
		return false;
	}
	else 
	{
		LOG_INF("GPIO gpio_pin_configure_dt led success");
	}

	return true;
}

//RELAY CONTROL initalization
bool init_relayctl(void)
{
	int ret;

	if (!gpio_is_ready_dt(&relayctl)) 
	{
		LOG_WRN("GPIO relayctl is not ready");
		return false;
	}
	else 
	{
		LOG_INF("GPIO relayctl is ready");
	}

	ret = gpio_pin_configure_dt(&relayctl, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) 
	{
	 	LOG_WRN("GPIO gpio_pin_configure_dt relayctl failed");
		return false;
	}
	else 
	{
		LOG_INF("GPIO gpio_pin_configure_dt relayctl success");
	}

	ret = gpio_pin_set_dt(&relayctl, 0); //Open Relay control
	if (ret != 0)
	{
		LOG_WRN("GPIO gpio_pin_set_dt relayctl failed");
		return false;
	}
	else 
	{
		LOG_INF("GPIO gpio_pin_set_dt relayctl success");
	}

	return true;
}


// Hardware Initialization
bool init_hw(void)
{

	// SOM LED
	if(!init_led0()) 
	{
		LOG_WRN("init_led0 failed");
		return false;
	}
	else 
	{
		LOG_INF("init_led0 success");
	}

	// RELAY CONTROL
	if(!init_relayctl()) 
	{
		LOG_WRN("init_relayctl failed");
		return false;
	}
	else 
	{
		LOG_INF("init_relayctl success");
	}

 return true;
}


int main(void)
{
	LOG_INF("========== Zephyr - M4F firmware ti-am62x-m4-firmware V0.3 (10/02/2025) ==========");

	//FLE:Adding for debugging:--------------------------------------------------
	//volatile unsigned int u8HaltCPU;
	//pMailboxHw = MAILBOX_REGBASE;
	//u8HaltCPU = 0;
	//while (u8HaltCPU == 0);// For debugging: Used to halt the runtime execution
	//---------------------------------------------------------------------------

	//Hardware Init
	if (false == init_hw())
	{//Hardware initialization failure
	    LOG_ERR("Hardware initialization failed");
		return 0;
	}
	else
	{
		LOG_INF("Hardware initialization successful");
	}
	
	//Start Openamp
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);

	k_thread_create(&thread_LinuxMsg_data, thread_LinuxMsg_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_linuxmsg,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	return 0;
}
