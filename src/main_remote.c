/*
 * Copyright (c) 2020, STMICROELECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

//FLE adding:
#include "pb_codec/nanopb/pb_decode.h"
#include "pb_codec/high_to_low.pb.h"
#include <zephyr/drivers/mbox.h>
#define CHANNEL_A53_TO_M4F (1) // channel = MailBox1 for A53 -> M4F
#define CHANNEL_M4F_TO_A53 (0) // channel = MailBox0 for M4F -> A53
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

/* Add 1024 extra bytes for the TTY task stack for the "tx_buff" buffer. */
#define APP_TTY_TASK_STACK_SIZE (1536)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rp__client_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, APP_TTY_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_rp__client_data;
static struct k_thread thread_tty_data;

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

static char rx_sc_msg[20];  /* should receive "Hello world!" */
static struct rpmsg_endpoint sc_ept;
static struct rpmsg_rcv_msg sc_msg = {.data = rx_sc_msg};

static struct rpmsg_endpoint tty_ept;
static struct rpmsg_rcv_msg tty_msg;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_sc_sem, 0, 1);
static K_SEM_DEFINE(data_tty_sem, 0, 1);

//FLE: adding:-----
/* Enum definitions */
#if 0
typedef enum _PWMState {
    PWMState_ON = 0,
    PWMState_OFF = 1,
    PWMState_F = 2
} PWMState;

typedef enum _SLACState {
    SLACState_UDF = 0,
    SLACState_RUN = 1,
    SLACState_OK = 2,
    SLACState_NOK = 3
} SLACState;

/* Struct definitions */
typedef struct _CpuHeartbeat {
    char dummy_field;
} CpuHeartbeat;

typedef struct _Empty {
    char dummy_field;
} Empty;

typedef struct _SetPWM {
    PWMState state;
    float duty_cycle;
} SetPWM;

typedef struct _SetSLAC {
    SLACState state;
} SetSLAC;
typedef uint_least16_t pb_size_t;
typedef struct _HighToLow {
    pb_size_t which_message;
    union {
        SetPWM set_pwm;
        bool allow_power_on;
        Empty enable;
        Empty disable;
        CpuHeartbeat heartbeat;
        SetSLAC set_slac;
    } message;
} HighToLow;
#endif
K_THREAD_STACK_DEFINE(thread_LinuxMsg_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_LinuxMsg_data;
static K_SEM_DEFINE(data_linuxmsg_sem, 0, 1);
static struct rpmsg_endpoint linuxmsg_ept;


#define OMAP_MAILBOX_NUM_MSGS  16
#define MAILBOX_MAX_CHANNELS   16
#define OMAP_MAILBOX_NUM_USERS 4
#define MAILBOX_MBOX_SIZE      sizeof(uint32_t)

#define MAILBOX_REGBASE (struct sMailboxHw *)0xA9000000

struct omap_mailbox_irq_regs {
	uint32_t status_raw;
	uint32_t status_clear;
	uint32_t enable_set;
	uint32_t enable_clear;
};

struct sMailboxHw {
	uint32_t revision;
	uint32_t __pad0[3];
	uint32_t sysconfig;
	uint32_t __pad1[11];
	uint32_t message[OMAP_MAILBOX_NUM_MSGS];
	uint32_t fifo_status[OMAP_MAILBOX_NUM_MSGS];
	uint32_t msg_status[OMAP_MAILBOX_NUM_MSGS];
	struct omap_mailbox_irq_regs irq_regs[OMAP_MAILBOX_NUM_USERS];
};

#define NB_MAILBOX_HW_MSG (10) //Messages received from Mailbox Hw
volatile unsigned int u8HaltCPU;
volatile unsigned long ClbkCounter = 0L;
struct fw_rsc_vdev_vring *vring0, *vring1;
volatile struct sMailboxHw *pMailboxHw; //For debugging: Map structure on Mailbox registers
uint32_t MailboxHw_Msg_rcv[NB_MAILBOX_HW_MSG];


/* Protobuf */
//#define HighToLow_size  (9)
uint8_t msg_buffer[HighToLow_size];
uint16_t msg_buffer_len = sizeof(msg_buffer);
static struct rpmsg_rcv_msg linuxmsg_msg = {.data = (void*)&msg_buffer};
HighToLow rx_linuxmsg_msg;

/* PWM */
float set_pwm_DC_given = 0.05; //to not have 0, which will put fsm into error state



static void mbox_callback(const struct device *dev, uint32_t channel,
		     void *user_data, struct mbox_msg *data)
{
	if (ClbkCounter < NB_MAILBOX_HW_MSG)
	{
		MailboxHw_Msg_rcv[ClbkCounter] = (uint32_t)*(uint32_t*)(data->data); //Msg in Hw Mailbox
	}

	ClbkCounter++;//For debugging: Update the Interrution counter 
	k_sem_give(&data_sem);
}

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
        LOG_INF("power_on msg received, enable variable = %d",intEnable);
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

//--------------

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("%s: msg received from mb %d", __func__, id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_cs_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	memcpy(sc_msg.data, data, len);
	sc_msg.len = len;
	k_sem_give(&data_sc_sem);

	return RPMSG_SUCCESS;
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = priv;

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_tty_sem);

	return RPMSG_SUCCESS;
}


static int rpmsg_recv_linuxmsg_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	//LOG_DBG("%s: Linux msg received !!!", __func__);
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
	//ipm_send(ipm_handle, 0, id, NULL, 0); FLE
	mbox_send(ipm_handle, CHANNEL_M4F_TO_A53, NULL); //FLE

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

	//ipm_register_callback(ipm_handle, platform_ipm_callback, NULL); FLE
	if (mbox_register_callback(ipm_handle, CHANNEL_A53_TO_M4F, mbox_callback, NULL) != 0) //FLE
	{
		LOG_ERR("ipm_register_callback failed");
	}
	
	LOG_DBG("ipm_register_callback OK");

	//status = ipm_set_enabled(ipm_handle, 1); FLE
	status = mbox_set_enabled(ipm_handle, CHANNEL_A53_TO_M4F, true); //FLE
	if (status) {
		LOG_ERR("ipm_set_enabled failed");
		return -1;
	}
	LOG_INF("ipm_set_enabled OK");

	return 0;
}

static void cleanup_system(void)
{
	//ipm_set_enabled(ipm_handle, 0); FLE
	mbox_set_enabled(ipm_handle, CHANNEL_A53_TO_M4F, false); //FLE
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

	vring_rsc = vring0 = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}
	LOG_INF("Ok to init vring 0: 0x%X",(unsigned int)vring0);

	vring_rsc = vring1 = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}
	LOG_INF("Ok to init vring 1: 0x%X",(unsigned int)vring1);

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

void app_rpmsg_client_sample(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0;

	k_sem_take(&data_sc_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux sample client responder started");

	ret = rpmsg_create_ept(&sc_ept, rpdev, "rpmsg-client-sample",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_cs_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux sample client] Could not create endpoint: %d", ret);
		goto task_end;
	}
	LOG_INF("[Linux sample client] OK to create endpoint");

	while (msg_cnt < 100) {
		k_sem_take(&data_sc_sem,  K_FOREVER);
		msg_cnt++;
		LOG_INF("[Linux sample client] incoming msg %d: %.*s", msg_cnt, sc_msg.len,
			(char *)sc_msg.data);
		rpmsg_send(&sc_ept, sc_msg.data, sc_msg.len);
	}
	rpmsg_destroy_ept(&sc_ept);

task_end:
	LOG_INF("OpenAMP Linux sample client responder ended");
}

void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char tx_buff[512];
	int ret = 0;

	k_sem_take(&data_tty_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux TTY responder started");

	tty_ept.priv = &tty_msg;
	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux TTY] Could not create endpoint: %d", ret);
		goto task_end;
	}
	LOG_INF("[Linux TTY] OK to create endpoint");

	while (tty_ept.addr !=  RPMSG_ADDR_ANY) {
		k_sem_take(&data_tty_sem,  K_FOREVER);
		if (tty_msg.len) {
			LOG_INF("[Linux TTY] incoming msg: %.*s",
				(int)tty_msg.len, (char *)tty_msg.data);
			snprintf(tx_buff, 13, "TTY 0x%04x: ", tty_ept.addr);
			memcpy(&tx_buff[12], tty_msg.data, tty_msg.len);
			rpmsg_send(&tty_ept, tx_buff, tty_msg.len + 12);
			rpmsg_release_rx_buffer(&tty_ept, tty_msg.data);
		}
		tty_msg.len = 0;
		tty_msg.data = NULL;
	}
	rpmsg_destroy_ept(&tty_ept);

task_end:
	LOG_INF("OpenAMP Linux TTY responder ended");
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder demo started");

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
	k_sem_give(&data_sc_sem);
	k_sem_give(&data_tty_sem);
	k_sem_give(&data_linuxmsg_sem);
	

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}


void app_rpmsg_linuxmsg(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0, k_sem_status;
	uint64_t last_heartbeat_ts, current_ts;
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

	last_heartbeat_ts = sys_clock_tick_get();
	while (1) {
		//k_sem_take(&data_linuxmsg_sem,  K_FOREVER);
		k_sem_status = k_sem_take(&data_linuxmsg_sem,  K_MSEC(500));
		if (k_sem_status == 0)
		{
			msg_cnt++;
			LOG_INF("[Linux message client] ==> Incoming msg %d: %.d ", msg_cnt, linuxmsg_msg.len);
			{
				unsigned int i;
				char *p = (char*)linuxmsg_msg.data;

				for (i=0; i<linuxmsg_msg.len; i++)
				{
					LOG_INF("Message[%i] = %x",i, p[i]);
				}

			}
			/* Protobuf decoding: 
			   msg_buffer contains the encoded linux message (received from IPC/mailbox)
			   rx_linuxmsg_msg contains the decoded linux message
			*/ 
			istream = pb_istream_from_buffer(msg_buffer, msg_buffer_len);
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

        if (k_ms_to_ticks_floor64((uint64_t)(3000)) < (current_ts - last_heartbeat_ts)) 
		{
           last_heartbeat_ts = current_ts;
 
 		 	LOG_INF("[Linux message client] Send Heartbeat");
 			//rpmsg_send(&linuxmsg_ept, sc_msg.data, sc_msg.len);
        }
	
	}
	rpmsg_destroy_ept(&linuxmsg_ept);

task_end:
	LOG_INF("OpenAMP[remote] Linux message responder ended");
}

int main(void)
{
	LOG_INF("Starting application threads!");

	//FLE:Adding
	pMailboxHw = MAILBOX_REGBASE;
	memset(MailboxHw_Msg_rcv, 0xFF, sizeof(MailboxHw_Msg_rcv));
	u8HaltCPU = 0;
	//while (u8HaltCPU == 0);// For debugging: Used to halt the runtime execution
	LOG_INF("ipm_handle: 0x%X",(unsigned int)ipm_handle);
	//---------------------------

	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	/*k_thread_create(&thread_rp__client_data, thread_rp__client_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_client_sample,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_create(&thread_tty_data, thread_tty_stack, APP_TTY_TASK_STACK_SIZE,
			app_rpmsg_tty,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);*/

	k_thread_create(&thread_LinuxMsg_data, thread_LinuxMsg_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_linuxmsg,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	return 0;
}
