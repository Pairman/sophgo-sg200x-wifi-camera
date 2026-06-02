#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mov-reader.h"
#include "mp4-writer.h"

typedef struct {
	FILE *fp;
} file_ctx_t;

typedef struct {
	uint32_t in_track;
	int out_track;
	uint8_t object;
	bool is_audio;
	bool is_video;
} track_map_t;

typedef struct {
	struct mp4_writer_t *writer;
	track_map_t tracks[8];
	int track_count;
	bool include_audio;
	int64_t start_ms;
	int64_t end_ms;
	int64_t first_pts[8];
	int64_t first_dts[8];
	bool wrote_sample;
} clip_ctx_t;

typedef struct {
	void *ptr;
	size_t bytes;
	uint32_t track;
	int64_t pts;
	int64_t dts;
	int flags;
} packet_t;

static int file_read(void *param, void *data, uint64_t bytes)
{
	return fread(data, 1, bytes, param) == bytes ? 0 : ferror(param);
}

static int file_write(void *param, const void *data, uint64_t bytes)
{
	return fwrite(data, 1, bytes, param) == bytes ? 0 : ferror(param);
}

static int file_seek(void *param, int64_t offset)
{
	return fseek(param, offset, offset >= 0 ? SEEK_SET : SEEK_END);
}

static int64_t file_tell(void *param)
{
	return ftell(param);
}

static const struct mov_buffer_t g_file_buffer = {
	.read = file_read,
	.write = file_write,
	.seek = file_seek,
	.tell = file_tell,
};

static void usage(const char *program)
{
	fprintf(stderr,
		"Usage: %s input.mp4 output.mp4 --start-ms <ms> --end-ms <ms> [--no-audio]\n",
		program);
}

static bool parse_long(const char *text, int64_t *value)
{
	char *end;

	errno = 0;
	*value = strtoll(text, &end, 10);
	return errno == 0 && end != text && *end == '\0';
}

static track_map_t *find_track(clip_ctx_t *ctx, uint32_t in_track)
{
	for (int i = 0; i < ctx->track_count; ++i) {
		if (ctx->tracks[i].in_track == in_track)
			return &ctx->tracks[i];
	}
	return NULL;
}

static void add_track(clip_ctx_t *ctx, uint32_t track, uint8_t object,
	int out_track, bool is_video, bool is_audio)
{
	track_map_t *map;

	if (ctx->track_count >= (int)(sizeof(ctx->tracks) / sizeof(ctx->tracks[0])))
		return;
	map = &ctx->tracks[ctx->track_count++];
	map->in_track = track;
	map->out_track = out_track;
	map->object = object;
	map->is_video = is_video;
	map->is_audio = is_audio;
}

static void on_video(void *param, uint32_t track, uint8_t object, int width, int height,
	const void *extra, size_t bytes)
{
	clip_ctx_t *ctx = param;
	int out_track;

	if (object != MOV_OBJECT_H264)
		return;
	out_track = mp4_writer_add_video(ctx->writer, object, width, height, extra, bytes);
	if (out_track >= 0)
		add_track(ctx, track, object, out_track, true, false);
}

static void on_audio(void *param, uint32_t track, uint8_t object, int channel_count,
	int bit_per_sample, int sample_rate, const void *extra, size_t bytes)
{
	clip_ctx_t *ctx = param;
	int out_track;

	if (!ctx->include_audio || object != MOV_OBJECT_AAC)
		return;
	out_track = mp4_writer_add_audio(ctx->writer, object, channel_count, bit_per_sample,
		sample_rate, extra, bytes);
	if (out_track >= 0)
		add_track(ctx, track, object, out_track, false, true);
}

static void on_subtitle(void *param, uint32_t track, uint8_t object,
	const void *extra, size_t bytes)
{
	(void)param;
	(void)track;
	(void)object;
	(void)extra;
	(void)bytes;
}

static void *on_alloc(void *param, uint32_t track, size_t bytes, int64_t pts,
	int64_t dts, int flags)
{
	packet_t *pkt = param;

	pkt->ptr = malloc(bytes);
	if (pkt->ptr == NULL)
		return NULL;
	pkt->bytes = bytes;
	pkt->track = track;
	pkt->pts = pts;
	pkt->dts = dts;
	pkt->flags = flags;
	return pkt->ptr;
}

static int write_packet(clip_ctx_t *ctx, const packet_t *pkt)
{
	track_map_t *track = find_track(ctx, pkt->track);
	int index;
	int64_t pts;
	int64_t dts;

	if (track == NULL)
		return 0;
	if (pkt->pts > ctx->end_ms)
		return 1;

	index = (int)(track - ctx->tracks);
	if (ctx->first_pts[index] < 0)
		ctx->first_pts[index] = pkt->pts;
	if (ctx->first_dts[index] < 0)
		ctx->first_dts[index] = pkt->dts;

	pts = pkt->pts - ctx->first_pts[index];
	dts = pkt->dts - ctx->first_dts[index];
	if (pts < 0)
		pts = 0;
	if (dts < 0)
		dts = 0;

	if (mp4_writer_write(ctx->writer, track->out_track, pkt->ptr, pkt->bytes,
	    pts, dts, pkt->flags) != 0)
		return -1;
	ctx->wrote_sample = true;
	return 0;
}

static int clip_copy(const char *input, const char *output, int64_t start_ms,
	int64_t end_ms, bool include_audio)
{
	FILE *ifp = NULL;
	FILE *ofp = NULL;
	mov_reader_t *reader = NULL;
	clip_ctx_t ctx;
	struct mov_reader_trackinfo_t info;
	int64_t seek_ms = start_ms;
	int ret = 1;

	memset(&ctx, 0, sizeof(ctx));
	ctx.include_audio = include_audio;
	ctx.start_ms = start_ms;
	ctx.end_ms = end_ms;
	for (int i = 0; i < (int)(sizeof(ctx.first_pts) / sizeof(ctx.first_pts[0])); ++i) {
		ctx.first_pts[i] = -1;
		ctx.first_dts[i] = -1;
	}

	ifp = fopen(input, "rb");
	if (ifp == NULL) {
		fprintf(stderr, "open input failed: %s\n", strerror(errno));
		goto out;
	}
	ofp = fopen(output, "wb+");
	if (ofp == NULL) {
		fprintf(stderr, "open output failed: %s\n", strerror(errno));
		goto out;
	}

	reader = mov_reader_create(&g_file_buffer, ifp);
	if (reader == NULL) {
		fprintf(stderr, "create mov reader failed\n");
		goto out;
	}

	ctx.writer = mp4_writer_create(0, &g_file_buffer, ofp, MOV_FLAG_FASTSTART);
	if (ctx.writer == NULL) {
		fprintf(stderr, "create mp4 writer failed\n");
		goto out;
	}

	info.onvideo = on_video;
	info.onaudio = on_audio;
	info.onsubtitle = on_subtitle;
	if (mov_reader_getinfo(reader, &info, &ctx) != 0 || ctx.track_count == 0) {
		fprintf(stderr, "read track info failed\n");
		goto out;
	}

	if (mov_reader_seek(reader, &seek_ms) != 0) {
		fprintf(stderr, "seek failed\n");
		goto out;
	}

	while (1) {
		packet_t pkt;
		int r;

		memset(&pkt, 0, sizeof(pkt));
		r = mov_reader_read2(reader, on_alloc, &pkt);
		if (r == 0) {
			ret = 0;
			break;
		}
		if (r < 0) {
			fprintf(stderr, "read packet failed\n");
			goto out;
		}

		r = write_packet(&ctx, &pkt);
		free(pkt.ptr);
		if (r < 0) {
			fprintf(stderr, "write packet failed\n");
			goto out;
		}
		if (r > 0) {
			ret = 0;
			break;
		}
	}

	if (!ctx.wrote_sample) {
		fprintf(stderr, "no sample written\n");
		ret = 1;
	}

out:
	if (ctx.writer != NULL)
		mp4_writer_destroy(ctx.writer);
	if (reader != NULL)
		mov_reader_destroy(reader);
	if (ofp != NULL)
		fclose(ofp);
	if (ifp != NULL)
		fclose(ifp);
	return ret;
}

int main(int argc, char **argv)
{
	const char *input;
	const char *output;
	int64_t start_ms = -1;
	int64_t end_ms = -1;
	bool include_audio = true;

	if (argc < 7) {
		usage(argv[0]);
		return 1;
	}

	input = argv[1];
	output = argv[2];
	for (int i = 3; i < argc; ++i) {
		if (strcmp(argv[i], "--start-ms") == 0 && i + 1 < argc) {
			if (!parse_long(argv[++i], &start_ms)) {
				usage(argv[0]);
				return 1;
			}
		} else if (strcmp(argv[i], "--end-ms") == 0 && i + 1 < argc) {
			if (!parse_long(argv[++i], &end_ms)) {
				usage(argv[0]);
				return 1;
			}
		} else if (strcmp(argv[i], "--no-audio") == 0) {
			include_audio = false;
		} else {
			usage(argv[0]);
			return 1;
		}
	}

	if (start_ms < 0 || end_ms <= start_ms) {
		usage(argv[0]);
		return 1;
	}

	return clip_copy(input, output, start_ms, end_ms, include_audio);
}
