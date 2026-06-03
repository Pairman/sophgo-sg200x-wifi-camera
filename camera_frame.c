#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "cvi_ae.h"
#include "cvi_ae_comm.h"
#include "cvi_awb.h"
#include "cvi_awb_comm.h"
#include "cvi_buffer.h"
#include "cvi_comm_isp.h"
#include "cvi_comm_sns.h"
#include "cvi_isp.h"
#include "cvi_sns_ctrl.h"
#include "cvi_venc.h"
#include "sample_comm.h"

#define FRAME_VI_DEV 0
#define FRAME_VI_CHN 0
#define FRAME_VENC_CHN 0
#define FRAME_QUALITY 90
#define OPT_SENSOR_TYPE GCORE_GC4653_MIPI_720P_60FPS_10BIT
#define GC4653_VTS_H_ADDR 0x0340
#define GC4653_VTS_L_ADDR 0x0341
#define GC4653_720P25_VTS 3600

extern int gc4653_write_register(VI_PIPE ViPipe, int addr, int data);
extern int gc4653_read_register(VI_PIPE ViPipe, int addr);

static SAMPLE_VI_CONFIG_S g_vi_config;

static bool g_sys_started;
static bool g_vi_started;
static bool g_venc_created;
static bool g_venc_receiving;
static bool g_venc_fd_open;
static CVI_S32 g_venc_fd = -1;

static void cleanup(void)
{
	if (g_venc_fd_open) {
		CVI_S32 ret = CVI_VENC_CloseFd(FRAME_VENC_CHN);
		if (ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_CloseFd failed: %#x\n", ret);
		g_venc_fd_open = false;
		g_venc_fd = -1;
	}

	if (g_venc_receiving) {
		CVI_S32 ret = CVI_VENC_StopRecvFrame(FRAME_VENC_CHN);
		if (ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_StopRecvFrame failed: %#x\n", ret);
		g_venc_receiving = false;

		ret = CVI_VENC_ResetChn(FRAME_VENC_CHN);
		if (ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_ResetChn failed: %#x\n", ret);
	}

	if (g_venc_created) {
		CVI_S32 ret = CVI_VENC_DestroyChn(FRAME_VENC_CHN);
		if (ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_DestroyChn failed: %#x\n", ret);
		g_venc_created = false;
	}

	if (g_vi_started) {
		SAMPLE_COMM_VI_DestroyIsp(&g_vi_config);
		SAMPLE_COMM_VI_DestroyVi(&g_vi_config);
		g_vi_started = false;
	}

	if (g_sys_started) {
		SAMPLE_COMM_SYS_Exit();
		g_sys_started = false;
	}

	memset(&g_vi_config, 0, sizeof(g_vi_config));
}

static void exit_with_cleanup(int code)
{
	cleanup();
	exit(code);
}

static void signal_handler(int signal_no, siginfo_t *siginfo, void *context)
{
	UNUSED(signal_no);
	UNUSED(siginfo);
	UNUSED(context);
	exit_with_cleanup(1);
}

static CVI_S32 install_signal_handlers(void)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	sigemptyset(&action.sa_mask);
	action.sa_sigaction = signal_handler;
	action.sa_flags = SA_SIGINFO | SA_RESETHAND;

	if (sigaction(SIGINT, &action, NULL) != 0)
		return CVI_FAILURE;
	if (sigaction(SIGTERM, &action, NULL) != 0)
		return CVI_FAILURE;

	return CVI_SUCCESS;
}

static CVI_S32 camera_init(CVI_U32 *width, CVI_U32 *height)
{
	MMF_VERSION_S version;
	SAMPLE_VI_CONFIG_S vi_config;
	SAMPLE_INI_CFG_S ini_config;
	PIC_SIZE_E pic_size;
	SIZE_S size;
	LOG_LEVEL_CONF_S log_conf;
	VB_CONFIG_S vb_config;
	CVI_U32 blk_size;
	CVI_U32 blk_rot_size;
	CVI_S32 ret;

	CVI_SYS_GetVersion(&version);
	SAMPLE_PRT("MMF Version:%s\n", version.version);

	log_conf.enModId = CVI_ID_LOG;
	log_conf.s32Level = CVI_DBG_INFO;
	CVI_LOG_SetLevelConf(&log_conf);

	ret = install_signal_handlers();
	if (ret != CVI_SUCCESS)
		return ret;

	ret = SAMPLE_COMM_VI_ParseIni(&ini_config);
	if (ret != CVI_SUCCESS)
		return ret;

	ini_config.enSnsType[0] = OPT_SENSOR_TYPE;
	SAMPLE_PRT("Override sensor mode: GCORE_GC4653_MIPI_720P_60FPS_10BIT\n");
	CVI_VI_SetDevNum(ini_config.devNum);

	ret = SAMPLE_COMM_VI_IniToViCfg(&ini_config, &vi_config);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = SAMPLE_COMM_VI_GetSizeBySensor(ini_config.enSnsType[0], &pic_size);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = SAMPLE_COMM_SYS_GetPicSize(pic_size, &size);
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&vb_config, 0, sizeof(vb_config));
	vb_config.u32MaxPoolCnt = 1;
	blk_size = COMMON_GetPicBufferSize(size.u32Width, size.u32Height,
		SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	blk_rot_size = COMMON_GetPicBufferSize(size.u32Height, size.u32Width,
		SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	vb_config.astCommPool[0].u32BlkSize = blk_size > blk_rot_size ? blk_size : blk_rot_size;
	vb_config.astCommPool[0].u32BlkCnt = 2;
	vb_config.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;

	ret = SAMPLE_COMM_SYS_Init(&vb_config);
	if (ret != CVI_SUCCESS)
		return ret;
	g_sys_started = true;

	ret = SAMPLE_PLAT_VI_INIT(&vi_config);
	if (ret != CVI_SUCCESS)
		return ret;
	g_vi_started = true;

	memcpy(&g_vi_config, &vi_config, sizeof(g_vi_config));
	*width = size.u32Width;
	*height = size.u32Height;

	return CVI_SUCCESS;
}

static CVI_S32 set_sensor_25fps(void)
{
	CVI_S32 ret;

	SAMPLE_PRT("GC4653 VTS before 25fps set: 0x%02x 0x%02x\n",
		gc4653_read_register(0, GC4653_VTS_H_ADDR) & 0xff,
		gc4653_read_register(0, GC4653_VTS_L_ADDR) & 0xff);

	ret = gc4653_write_register(0, GC4653_VTS_H_ADDR, (GC4653_720P25_VTS >> 8) & 0xff);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = gc4653_write_register(0, GC4653_VTS_L_ADDR, GC4653_720P25_VTS & 0xff);
	if (ret != CVI_SUCCESS)
		return ret;

	SAMPLE_PRT("GC4653 VTS after 25fps set: 0x%02x 0x%02x\n",
		gc4653_read_register(0, GC4653_VTS_H_ADDR) & 0xff,
		gc4653_read_register(0, GC4653_VTS_L_ADDR) & 0xff);

	return CVI_SUCCESS;
}

static CVI_S32 venc_init(CVI_U32 width, CVI_U32 height)
{
	VENC_CHN_ATTR_S chn_attr;
	VENC_JPEG_PARAM_S jpeg_param;
	VENC_RECV_PIC_PARAM_S recv_param;
	CVI_S32 ret;

	memset(&chn_attr, 0, sizeof(chn_attr));
	chn_attr.stVencAttr.enType = PT_JPEG;
	chn_attr.stVencAttr.u32MaxPicWidth = width;
	chn_attr.stVencAttr.u32MaxPicHeight = height;
	chn_attr.stVencAttr.u32PicWidth = width;
	chn_attr.stVencAttr.u32PicHeight = height;
	chn_attr.stVencAttr.u32BufSize = width * height;
	chn_attr.stVencAttr.bEsBufQueueEn = CVI_FALSE;
	chn_attr.stVencAttr.bIsoSendFrmEn = CVI_FALSE;
	chn_attr.stVencAttr.bByFrame = CVI_TRUE;
	chn_attr.stVencAttr.stAttrJpege.bSupportDCF = CVI_FALSE;
	chn_attr.stVencAttr.stAttrJpege.stMPFCfg.u8LargeThumbNailNum = 0;
	chn_attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
	chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
	chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	chn_attr.stGopAttr.stNormalP.s32IPQpDelta = 0;

	ret = CVI_VENC_CreateChn(FRAME_VENC_CHN, &chn_attr);
	if (ret != CVI_SUCCESS)
		return ret;
	g_venc_created = true;

	memset(&jpeg_param, 0, sizeof(jpeg_param));
	ret = CVI_VENC_GetJpegParam(FRAME_VENC_CHN, &jpeg_param);
	if (ret != CVI_SUCCESS)
		return ret;

	jpeg_param.u32Qfactor = FRAME_QUALITY;
	jpeg_param.u32MCUPerECS = 0;
	ret = CVI_VENC_SetJpegParam(FRAME_VENC_CHN, &jpeg_param);
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&recv_param, 0, sizeof(recv_param));
	recv_param.s32RecvPicNum = 1;
	ret = CVI_VENC_StartRecvFrame(FRAME_VENC_CHN, &recv_param);
	if (ret != CVI_SUCCESS)
		return ret;
	g_venc_receiving = true;

	g_venc_fd = CVI_VENC_GetFd(FRAME_VENC_CHN);
	if (g_venc_fd < 0)
		return CVI_FAILURE;
	g_venc_fd_open = true;

	return CVI_SUCCESS;
}

static CVI_S32 wait_venc_ready(void)
{
	fd_set read_fds;
	struct timeval timeout;
	CVI_S32 ret;

	FD_ZERO(&read_fds);
	FD_SET(g_venc_fd, &read_fds);
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	ret = select(g_venc_fd + 1, &read_fds, NULL, NULL, &timeout);
	if (ret < 0) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "select VENC fd failed: %s\n", strerror(errno));
		return CVI_FAILURE;
	}
	if (ret == 0 || !FD_ISSET(g_venc_fd, &read_fds)) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "select VENC fd timeout\n");
		return CVI_FAILURE;
	}

	return CVI_SUCCESS;
}

static CVI_S32 save_jpeg_stream(const char *output_path)
{
	VENC_CHN_STATUS_S status;
	VENC_STREAM_S stream;
	FILE *output;
	CVI_S32 ret;

	ret = wait_venc_ready();
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&status, 0, sizeof(status));
	ret = CVI_VENC_QueryStatus(FRAME_VENC_CHN, &status);
	if (ret != CVI_SUCCESS)
		return ret;
	if (status.u32CurPacks == 0)
		return CVI_FAILURE;

	memset(&stream, 0, sizeof(stream));
	stream.pstPack = malloc(sizeof(VENC_PACK_S) * status.u32CurPacks);
	if (stream.pstPack == NULL)
		return CVI_FAILURE;

	ret = CVI_VENC_GetStream(FRAME_VENC_CHN, &stream, 1000);
	if (ret != CVI_SUCCESS)
		goto out_free_pack;

	output = fopen(output_path, "wb");
	if (output == NULL) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "fopen(%s) failed: %s\n", output_path, strerror(errno));
		ret = CVI_FAILURE;
		goto out_release_stream;
	}

	ret = SAMPLE_COMM_VENC_SaveStream(PT_JPEG, output, &stream);
	fclose(output);

out_release_stream:
	{
		CVI_S32 release_ret = CVI_VENC_ReleaseStream(FRAME_VENC_CHN, &stream);
		if (release_ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_ReleaseStream failed: %#x\n", release_ret);
	}
out_free_pack:
	free(stream.pstPack);
	return ret;
}

static CVI_S32 discard_warmup_frame(void)
{
	VIDEO_FRAME_INFO_S frame;
	CVI_S32 ret;

	memset(&frame, 0, sizeof(frame));
	ret = CVI_VI_GetChnFrame(FRAME_VI_DEV, FRAME_VI_CHN, &frame, 3000);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = CVI_VI_ReleaseChnFrame(FRAME_VI_DEV, FRAME_VI_CHN, &frame);
	if (ret != CVI_SUCCESS)
		CVI_TRACE_LOG(CVI_DBG_ERR, "warmup CVI_VI_ReleaseChnFrame failed: %#x\n", ret);

	return ret;
}

static CVI_S32 capture_frame(const char *output_path)
{
	VIDEO_FRAME_INFO_S frame;
	CVI_U32 width;
	CVI_U32 height;
	CVI_S32 ret;

	ret = camera_init(&width, &height);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = venc_init(width, height);
	if (ret != CVI_SUCCESS)
		return ret;

	usleep(800 * 1000);
	ret = discard_warmup_frame();
	if (ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "discard warmup frame failed: %#x\n", ret);
		return ret;
	}

	ret = set_sensor_25fps();
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&frame, 0, sizeof(frame));
	ret = CVI_VI_GetChnFrame(FRAME_VI_DEV, FRAME_VI_CHN, &frame, 3000);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = CVI_VENC_SendFrame(FRAME_VENC_CHN, &frame, 1000);
	if (ret == CVI_SUCCESS)
		ret = save_jpeg_stream(output_path);

	{
		CVI_S32 release_ret = CVI_VI_ReleaseChnFrame(FRAME_VI_DEV, FRAME_VI_CHN, &frame);
		if (release_ret != CVI_SUCCESS)
			CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VI_ReleaseChnFrame failed: %#x\n", release_ret);
	}

	return ret;
}

static void usage(const char *program)
{
	fprintf(stderr, "Usage: %s [-o output.jpg]\n", program);
}

int main(int argc, char **argv)
{
	char output_path[PATH_MAX];
	time_t now;
	CVI_S32 ret;
	int opt;

	now = time(NULL);
	if (now == (time_t)-1) {
		fprintf(stderr, "time() failed\n");
		return 1;
	}

	snprintf(output_path, sizeof(output_path), "frame_%lld.jpg", (long long)now);

	while ((opt = getopt(argc, argv, "o:")) != -1) {
		switch (opt) {
		case 'o':
			if (optarg[0] == '\0') {
				usage(argv[0]);
				return 1;
			}
			if (snprintf(output_path, sizeof(output_path), "%s", optarg) >= (int)sizeof(output_path)) {
				fprintf(stderr, "output path too long\n");
				return 1;
			}
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}
	if (optind != argc) {
		usage(argv[0]);
		return 1;
	}

	ret = capture_frame(output_path);
	cleanup();

	return ret == CVI_SUCCESS ? 0 : 1;
}
