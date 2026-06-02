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
#include "cvi_aacenc.h"
#include "cvi_buffer.h"
#include "cvi_comm_isp.h"
#include "cvi_comm_sns.h"
#include "cvi_isp.h"
#include "cvi_sns_ctrl.h"
#include "cvi_venc.h"
#include "fmp4-writer.h"
#include "mov-format.h"
#include "mpeg4-aac.h"
#include "mpeg4-avc.h"
#include "sample_comm.h"
#include "asoundlib.h"

#define VI_DEV 0
#define VI_CHN 0
#define VENC_CHN_ID 0
#define AUDIO_CARD 0
#define AUDIO_DEVICE 0

#define STREAM_WIDTH 1280
#define STREAM_HEIGHT 720
#define STREAM_FPS 25
#define STREAM_BITRATE_KBPS 4096
#define STREAM_FRAME_MS (1000 / STREAM_FPS)
#define AUDIO_SAMPLE_RATE 8000
#define AUDIO_CHANNELS 1
#define AUDIO_SAMPLES_PER_FRAME AACENC_BLOCKSIZE
#define AUDIO_BITRATE 32000
#define AUDIO_VOLUME 24
#define OPT_SENSOR_TYPE GCORE_GC4653_MIPI_720P_60FPS_10BIT
#define GC4653_VTS_H_ADDR 0x0340
#define GC4653_VTS_L_ADDR 0x0341
#define GC4653_720P25_VTS 3600

extern int gc4653_write_register(VI_PIPE ViPipe, int addr, int data);
extern int gc4653_read_register(VI_PIPE ViPipe, int addr);

static SAMPLE_VI_CONFIG_S g_vi_config;
static CVI_S32 g_venc_fd = -1;
static FILE *g_output;
static fmp4_writer_t *g_fmp4;
static struct pcm *g_audio_pcm;
static AAC_ENCODER_S *g_aac_encoder;
static struct mpeg4_avc_t g_avc;
static int g_fmp4_track = -1;
static int g_audio_track = -1;
static CVI_U32 g_fmp4_frame_index;
static CVI_U64 g_audio_sample_index;
static volatile sig_atomic_t g_stop;

static bool g_sys_ready;
static bool g_vi_ready;
static bool g_venc_ready;
static bool g_venc_receiving;
static bool g_venc_fd_open;

static void cleanup(void)
{
	if (g_fmp4 != NULL) {
		fmp4_writer_destroy(g_fmp4);
		g_fmp4 = NULL;
	}

	if (g_output != NULL) {
		fclose(g_output);
		g_output = NULL;
	}

	if (g_audio_pcm != NULL) {
		pcm_close(g_audio_pcm);
		g_audio_pcm = NULL;
	}

	if (g_aac_encoder != NULL) {
		AACEncoderClose(g_aac_encoder);
		g_aac_encoder = NULL;
	}

	if (g_venc_fd_open) {
		CVI_VENC_CloseFd(VENC_CHN_ID);
		g_venc_fd_open = false;
		g_venc_fd = -1;
	}

	if (g_venc_receiving) {
		CVI_VENC_StopRecvFrame(VENC_CHN_ID);
		g_venc_receiving = false;
	}

	if (g_venc_ready) {
		CVI_VENC_DestroyChn(VENC_CHN_ID);
		g_venc_ready = false;
	}

	if (g_vi_ready) {
		SAMPLE_COMM_VI_DestroyIsp(&g_vi_config);
		SAMPLE_COMM_VI_DestroyVi(&g_vi_config);
		g_vi_ready = false;
	}

	if (g_sys_ready) {
		SAMPLE_COMM_SYS_Exit();
		g_sys_ready = false;
	}

	memset(&g_vi_config, 0, sizeof(g_vi_config));
}

static void signal_handler(int signo, siginfo_t *siginfo, void *context)
{
	UNUSED(signo);
	UNUSED(siginfo);
	UNUSED(context);
	g_stop = 1;
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

static CVI_S32 init_sys_and_vi(void)
{
	SAMPLE_INI_CFG_S ini_config;
	SAMPLE_VI_CONFIG_S vi_config;
	PIC_SIZE_E pic_size;
	SIZE_S sensor_size;
	VB_CONFIG_S vb_config;
	MMF_VERSION_S version;
	LOG_LEVEL_CONF_S log_conf;
	CVI_U32 blk_size;
	CVI_U32 blk_rot_size;
	CVI_S32 ret;

	CVI_SYS_GetVersion(&version);
	SAMPLE_PRT("MMF Version:%s\n", version.version);

	log_conf.enModId = CVI_ID_LOG;
	log_conf.s32Level = CVI_DBG_INFO;
	CVI_LOG_SetLevelConf(&log_conf);

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

	ret = SAMPLE_COMM_SYS_GetPicSize(pic_size, &sensor_size);
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&vb_config, 0, sizeof(vb_config));
	vb_config.u32MaxPoolCnt = 1;
	blk_size = COMMON_GetPicBufferSize(sensor_size.u32Width, sensor_size.u32Height,
		SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	blk_rot_size = COMMON_GetPicBufferSize(sensor_size.u32Height, sensor_size.u32Width,
		SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
	vb_config.astCommPool[0].u32BlkSize = blk_size > blk_rot_size ? blk_size : blk_rot_size;
	vb_config.astCommPool[0].u32BlkCnt = 2;
	vb_config.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;

	ret = SAMPLE_COMM_SYS_Init(&vb_config);
	if (ret != CVI_SUCCESS)
		return ret;
	g_sys_ready = true;

	ret = SAMPLE_PLAT_VI_INIT(&vi_config);
	if (ret != CVI_SUCCESS)
		return ret;
	g_vi_ready = true;
	memcpy(&g_vi_config, &vi_config, sizeof(g_vi_config));

	return CVI_SUCCESS;
}

static CVI_S32 init_venc(int frame_count)
{
	VENC_CHN_ATTR_S attr;
	VENC_RECV_PIC_PARAM_S recv_param;
	CVI_S32 ret;

	memset(&attr, 0, sizeof(attr));
	attr.stVencAttr.enType = PT_H264;
	attr.stVencAttr.u32MaxPicWidth = STREAM_WIDTH;
	attr.stVencAttr.u32MaxPicHeight = STREAM_HEIGHT;
	attr.stVencAttr.u32PicWidth = STREAM_WIDTH;
	attr.stVencAttr.u32PicHeight = STREAM_HEIGHT;
	attr.stVencAttr.u32BufSize = STREAM_WIDTH * STREAM_HEIGHT;
	attr.stVencAttr.u32Profile = 66;
	attr.stVencAttr.bByFrame = CVI_TRUE;
	attr.stVencAttr.bSingleCore = CVI_TRUE;
	attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
	attr.stRcAttr.stH264Cbr.u32Gop = STREAM_FPS * 2;
	attr.stRcAttr.stH264Cbr.u32StatTime = 3;
	attr.stRcAttr.stH264Cbr.u32SrcFrameRate = STREAM_FPS;
	attr.stRcAttr.stH264Cbr.fr32DstFrameRate = STREAM_FPS;
	attr.stRcAttr.stH264Cbr.u32BitRate = STREAM_BITRATE_KBPS;
	attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	attr.stGopAttr.stNormalP.s32IPQpDelta = 2;

	ret = CVI_VENC_CreateChn(VENC_CHN_ID, &attr);
	if (ret != CVI_SUCCESS)
		return ret;
	g_venc_ready = true;

	memset(&recv_param, 0, sizeof(recv_param));
	recv_param.s32RecvPicNum = frame_count;
	ret = CVI_VENC_StartRecvFrame(VENC_CHN_ID, &recv_param);
	if (ret != CVI_SUCCESS)
		return ret;
	g_venc_receiving = true;

	g_venc_fd = CVI_VENC_GetFd(VENC_CHN_ID);
	if (g_venc_fd < 0)
		return CVI_FAILURE;
	g_venc_fd_open = true;

	return CVI_SUCCESS;
}

static CVI_S32 get_vi_frame(VIDEO_FRAME_INFO_S *frame)
{
	memset(frame, 0, sizeof(*frame));
	return CVI_VI_GetChnFrame(VI_DEV, VI_CHN, frame, 3000);
}

static void release_vi_frame(VIDEO_FRAME_INFO_S *frame)
{
	CVI_S32 ret = CVI_VI_ReleaseChnFrame(VI_DEV, VI_CHN, frame);
	if (ret != CVI_SUCCESS)
		CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VI_ReleaseChnFrame failed: %#x\n", ret);
}

static CVI_S32 wait_venc_stream(void)
{
	struct timeval timeout;
	fd_set read_fds;
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

static int mov_file_read(void *param, void *data, uint64_t bytes)
{
	return fread(data, 1, bytes, param) == bytes ? 0 : ferror(param);
}

static int mov_file_write(void *param, const void *data, uint64_t bytes)
{
	return fwrite(data, 1, bytes, param) == bytes ? 0 : ferror(param);
}

static int mov_file_seek(void *param, int64_t offset)
{
	return fseek(param, offset, offset >= 0 ? SEEK_SET : SEEK_END);
}

static int64_t mov_file_tell(void *param)
{
	return ftell(param);
}

static CVI_S32 init_fmp4_output(const char *path)
{
	static const struct mov_buffer_t buffer = {
		.read = mov_file_read,
		.write = mov_file_write,
		.seek = mov_file_seek,
		.tell = mov_file_tell,
	};

	g_output = fopen(path, "wb+");
	if (g_output == NULL) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "fopen(%s) failed: %s\n", path, strerror(errno));
		return CVI_FAILURE;
	}

	g_fmp4 = fmp4_writer_create(&buffer, g_output, 0);
	if (g_fmp4 == NULL)
		return CVI_FAILURE;

	memset(&g_avc, 0, sizeof(g_avc));
	g_fmp4_track = -1;
	g_fmp4_frame_index = 0;
	g_audio_sample_index = 0;
	return CVI_SUCCESS;
}

static CVI_S32 init_audio(void)
{
	AACENC_CONFIG aac_config;
	struct pcm_config config;
	struct mixer *mixer;
	struct mixer_ctl *ctl;
	CVI_S32 ret;

	mixer = mixer_open(AUDIO_CARD);
	if (mixer != NULL) {
		ctl = mixer_get_ctl_by_name(mixer, "ADC Capture Volume");
		if (ctl != NULL)
			mixer_ctl_set_value(ctl, 0, AUDIO_VOLUME);
		mixer_close(mixer);
	}

	memset(&config, 0, sizeof(config));
	config.channels = AUDIO_CHANNELS;
	config.rate = AUDIO_SAMPLE_RATE;
	config.period_size = AUDIO_SAMPLES_PER_FRAME;
	config.period_count = 4;
	config.format = PCM_FORMAT_S16_LE;

	g_audio_pcm = pcm_open(AUDIO_CARD, AUDIO_DEVICE, PCM_IN, &config);
	if (g_audio_pcm == NULL || !pcm_is_ready(g_audio_pcm)) {
		if (g_audio_pcm != NULL)
			CVI_TRACE_LOG(CVI_DBG_ERR, "pcm_open failed: %s\n", pcm_get_error(g_audio_pcm));
		return CVI_FAILURE;
	}

	ret = AACInitDefaultConfig(&aac_config);
	if (ret != CVI_SUCCESS)
		return ret;

	aac_config.quality = AU_QualityHigh;
	aac_config.coderFormat = AACLC;
	aac_config.bitsPerSample = 16;
	aac_config.sampleRate = AUDIO_SAMPLE_RATE;
	aac_config.bitRate = AUDIO_BITRATE;
	aac_config.nChannelsIn = AUDIO_CHANNELS;
	aac_config.nChannelsOut = AUDIO_CHANNELS;
	aac_config.bandWidth = AUDIO_SAMPLE_RATE / 2;
	aac_config.transtype = AACENC_ADTS;

	ret = AACEncoderOpen(&g_aac_encoder, &aac_config);
	if (ret != CVI_SUCCESS)
		return ret;

	return CVI_SUCCESS;
}

static CVI_S32 drain_audio_until(CVI_U64 max_pts_ms)
{
	CVI_S16 pcm_samples[AUDIO_SAMPLES_PER_FRAME];
	CVI_U8 encoded[20480];
	struct mpeg4_aac_t aac;
	CVI_U8 extra_data[64];
	int extra_data_size;
	CVI_U8 *payload;
	int adts_size;
	CVI_S32 encoded_bytes = 0;
	CVI_S32 ret;

	while ((g_audio_sample_index * 1000) / AUDIO_SAMPLE_RATE <= max_pts_ms) {
		ret = pcm_read(g_audio_pcm, pcm_samples, sizeof(pcm_samples));
		if (ret != CVI_SUCCESS)
			return ret;

		ret = AACEncoderFrame(g_aac_encoder, pcm_samples, encoded,
			sizeof(pcm_samples), &encoded_bytes);
		if (ret != CVI_SUCCESS)
			return ret;
		if (encoded_bytes <= 0)
			return CVI_FAILURE;

		adts_size = mpeg4_aac_adts_load(encoded, encoded_bytes, &aac);
		if (adts_size <= 0 || adts_size >= encoded_bytes)
			return CVI_FAILURE;

		if (g_audio_track < 0) {
			extra_data_size = mpeg4_aac_audio_specific_config_save(
				&aac, extra_data, sizeof(extra_data));
			if (extra_data_size <= 0)
				return CVI_FAILURE;

			g_audio_track = fmp4_writer_add_audio(g_fmp4, MOV_OBJECT_AAC,
				aac.channel_configuration, 16, AUDIO_SAMPLE_RATE,
				extra_data, extra_data_size);
			if (g_audio_track < 0)
				return CVI_FAILURE;
		}

		payload = encoded + adts_size;
		ret = fmp4_writer_write(g_fmp4, g_audio_track, payload, encoded_bytes - adts_size,
			(CVI_S64)(g_audio_sample_index * 1000 / AUDIO_SAMPLE_RATE),
			(CVI_S64)(g_audio_sample_index * 1000 / AUDIO_SAMPLE_RATE), 0);
		if (ret != CVI_SUCCESS)
			return ret;

		g_audio_sample_index += AUDIO_SAMPLES_PER_FRAME;
	}

	return CVI_SUCCESS;
}

static CVI_S32 write_fmp4_frame(const CVI_U8 *annexb, size_t annexb_bytes)
{
	CVI_U8 *sample;
	CVI_U8 extra_data[64 * 1024];
	int extra_data_bytes;
	int vcl = 0;
	int update = 0;
	int sample_bytes;
	CVI_S32 ret;

	sample = malloc(annexb_bytes + 64);
	if (sample == NULL)
		return CVI_FAILURE;

	sample_bytes = h264_annexbtomp4(&g_avc, annexb, annexb_bytes,
		sample, annexb_bytes + 64, &vcl, &update);
	if (sample_bytes <= 0) {
		ret = CVI_FAILURE;
		goto out;
	}

	if (g_fmp4_track < 0) {
		if (g_avc.nb_sps < 1 || g_avc.nb_pps < 1) {
			ret = CVI_FAILURE;
			goto out;
		}

		extra_data_bytes = mpeg4_avc_decoder_configuration_record_save(
			&g_avc, extra_data, sizeof(extra_data));
		if (extra_data_bytes <= 0) {
			ret = CVI_FAILURE;
			goto out;
		}

		g_fmp4_track = fmp4_writer_add_video(g_fmp4, MOV_OBJECT_H264,
			STREAM_WIDTH, STREAM_HEIGHT, extra_data, extra_data_bytes);
		if (g_fmp4_track < 0) {
			ret = CVI_FAILURE;
			goto out;
		}
	}

	ret = fmp4_writer_write(g_fmp4, g_fmp4_track, sample, sample_bytes,
		(CVI_S64)g_fmp4_frame_index * STREAM_FRAME_MS,
		(CVI_S64)g_fmp4_frame_index * STREAM_FRAME_MS,
		vcl == 1 ? MOV_AV_FLAG_KEYFREAME : 0);
	if (ret == CVI_SUCCESS)
		++g_fmp4_frame_index;

out:
	free(sample);
	return ret;
}

static CVI_S32 write_one_stream(void)
{
	VENC_CHN_STATUS_S status;
	VENC_STREAM_S stream;
	CVI_U8 *frame_data = NULL;
	size_t frame_bytes = 0;
	size_t offset = 0;
	CVI_S32 release_ret;
	CVI_S32 ret;

	ret = wait_venc_stream();
	if (ret != CVI_SUCCESS)
		return ret;

	memset(&status, 0, sizeof(status));
	ret = CVI_VENC_QueryStatus(VENC_CHN_ID, &status);
	if (ret != CVI_SUCCESS)
		return ret;
	if (status.u32CurPacks == 0)
		return CVI_FAILURE;

	memset(&stream, 0, sizeof(stream));
	stream.pstPack = malloc(sizeof(VENC_PACK_S) * status.u32CurPacks);
	if (stream.pstPack == NULL)
		return CVI_FAILURE;

	ret = CVI_VENC_GetStream(VENC_CHN_ID, &stream, 1000);
	if (ret != CVI_SUCCESS)
		goto out;

	for (CVI_U32 i = 0; i < stream.u32PackCount; ++i)
		frame_bytes += stream.pstPack[i].u32Len - stream.pstPack[i].u32Offset;

	frame_data = malloc(frame_bytes);
	if (frame_data == NULL) {
		ret = CVI_FAILURE;
		goto release_stream;
	}

	for (CVI_U32 i = 0; i < stream.u32PackCount; ++i) {
		VENC_PACK_S *pack = &stream.pstPack[i];
		size_t bytes = pack->u32Len - pack->u32Offset;

		memcpy(frame_data + offset, pack->pu8Addr + pack->u32Offset, bytes);
		offset += bytes;
	}

	ret = write_fmp4_frame(frame_data, frame_bytes);

release_stream:
	release_ret = CVI_VENC_ReleaseStream(VENC_CHN_ID, &stream);
	if (release_ret != CVI_SUCCESS) {
		CVI_TRACE_LOG(CVI_DBG_ERR, "CVI_VENC_ReleaseStream failed: %#x\n", release_ret);
		if (ret == CVI_SUCCESS)
			ret = release_ret;
	}

out:
	free(frame_data);
	free(stream.pstPack);
	return ret;
}

static CVI_S32 encode_one_frame(void)
{
	VIDEO_FRAME_INFO_S frame;
	CVI_S32 ret;

	ret = get_vi_frame(&frame);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = CVI_VENC_SendFrame(VENC_CHN_ID, &frame, 1000);
	release_vi_frame(&frame);
	if (ret != CVI_SUCCESS)
		return ret;

	return write_one_stream();
}

static CVI_S32 set_sensor_25fps(void)
{
	int vts_h;
	int vts_l;
	CVI_S32 ret;

	vts_h = gc4653_read_register(0, GC4653_VTS_H_ADDR);
	vts_l = gc4653_read_register(0, GC4653_VTS_L_ADDR);
	SAMPLE_PRT("GC4653 VTS before 25fps set: 0x%02x 0x%02x\n", vts_h & 0xff, vts_l & 0xff);

	ret = gc4653_write_register(0, GC4653_VTS_H_ADDR, (GC4653_720P25_VTS >> 8) & 0xff);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = gc4653_write_register(0, GC4653_VTS_L_ADDR, GC4653_720P25_VTS & 0xff);
	if (ret != CVI_SUCCESS)
		return ret;

	vts_h = gc4653_read_register(0, GC4653_VTS_H_ADDR);
	vts_l = gc4653_read_register(0, GC4653_VTS_L_ADDR);
	SAMPLE_PRT("GC4653 VTS after 25fps set: 0x%02x 0x%02x\n", vts_h & 0xff, vts_l & 0xff);

	return CVI_SUCCESS;
}

static CVI_S32 capture_stream(const char *path, int seconds)
{
	VIDEO_FRAME_INFO_S warmup_frame;
	int frame_count = STREAM_FPS * seconds;
	CVI_S32 ret;

	ret = init_sys_and_vi();
	if (ret != CVI_SUCCESS)
		return ret;

	ret = init_venc(frame_count);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = init_fmp4_output(path);
	if (ret != CVI_SUCCESS)
		return ret;

	ret = init_audio();
	if (ret != CVI_SUCCESS)
		return ret;

	usleep(800 * 1000);
	ret = get_vi_frame(&warmup_frame);
	if (ret == CVI_SUCCESS)
		release_vi_frame(&warmup_frame);
	if (ret != CVI_SUCCESS)
		goto out;

	ret = set_sensor_25fps();
	if (ret != CVI_SUCCESS)
		goto out;

	for (int i = 0; i < frame_count && !g_stop; ++i) {
		ret = encode_one_frame();
		if (ret != CVI_SUCCESS)
			goto out;

		ret = drain_audio_until((CVI_U64)(i + 1) * STREAM_FRAME_MS);
		if (ret != CVI_SUCCESS)
			goto out;
	}

	if (!g_stop)
		ret = drain_audio_until((CVI_U64)seconds * 1000);

out:
	return ret;
}

static void usage(const char *program)
{
	fprintf(stderr, "Usage: %s -s <seconds>\n", program);
}

int main(int argc, char **argv)
{
	char output_path[PATH_MAX];
	const char *requested_output = NULL;
	time_t now;
	char *end;
	long seconds = 0;
	int opt;
	CVI_S32 ret;

	while ((opt = getopt(argc, argv, "s:o:")) != -1) {
		switch (opt) {
		case 's':
			errno = 0;
			seconds = strtol(optarg, &end, 10);
			if (errno != 0 || *end != '\0' || seconds <= 0 || seconds > INT_MAX / STREAM_FPS) {
				usage(argv[0]);
				return 1;
			}
			break;
		case 'o':
			requested_output = optarg;
			break;
		default:
			usage(argv[0]);
			return 1;
		}
	}

	if (seconds == 0 || optind != argc) {
		usage(argv[0]);
		return 1;
	}

	ret = install_signal_handlers();
	if (ret != CVI_SUCCESS)
		return 1;

	if (requested_output != NULL) {
		snprintf(output_path, sizeof(output_path), "%s", requested_output);
	} else {
		now = time(NULL);
		if (now == (time_t)-1)
			return 1;
		snprintf(output_path, sizeof(output_path), "stream_%lld.mp4", (long long)now);
	}

	ret = capture_stream(output_path, seconds);
	cleanup();

	return ret == CVI_SUCCESS ? 0 : 1;
}
