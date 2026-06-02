#include <arpa/inet.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_SERVER_PORT 8001
#define APP_DIR "/var/lib/wifi-camera"
#define BIN_DIR APP_DIR "/bin"
#define SYSTEM_BIN_DIR "/usr/bin"
#define LIBEXEC_DIR "/usr/libexec/wifi-camera"
#define RECORDINGS_DIR APP_DIR "/recordings"
#define LOG_DIR "/var/log/wifi-camera"
#define LOW_SPACE_BYTES (2ULL * 1024ULL * 1024ULL * 1024ULL)
#define MAX_REQ 8192
#define MAX_BODY 4096
#define MAX_FILES 256
#define RECORD_SECONDS 31536000

typedef enum {
	REC_IDLE = 0,
	REC_STARTING,
	REC_RECORDING,
	REC_STOPPING,
	REC_ERROR,
} rec_state_t;

typedef enum {
	CLIP_NONE = 0,
	CLIP_RUNNING,
	CLIP_DONE,
	CLIP_FAILED,
} clip_state_t;

typedef struct {
	char name[NAME_MAX + 1];
	uint64_t size;
	time_t mtime;
} recording_t;

static volatile sig_atomic_t g_shutdown;
static int g_server_fd = -1;
static int g_server_port = DEFAULT_SERVER_PORT;
static rec_state_t g_rec_state = REC_IDLE;
static pid_t g_rec_pid = -1;
static char g_current_file[PATH_MAX];
static time_t g_started_at;
static char g_last_error[256];
static FILE *g_log;

static clip_state_t g_clip_state = CLIP_NONE;
static pid_t g_clip_pid = -1;
static char g_clip_input[NAME_MAX + 1];
static char g_clip_output[NAME_MAX + 1];
static char g_clip_error[256];
static time_t g_clip_started_at;
static time_t g_clip_finished_at;

static pid_t find_camera_stream_process(void);

static void set_last_error(const char *msg)
{
	snprintf(g_last_error, sizeof(g_last_error), "%s", msg != NULL ? msg : "");
}

static void log_msg(const char *fmt, ...)
{
	va_list ap;
	time_t now;
	struct tm tmv;
	char ts[32];

	if (g_log == NULL)
		return;

	now = time(NULL);
	localtime_r(&now, &tmv);
	strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmv);
	fprintf(g_log, "[%s] ", ts);

	va_start(ap, fmt);
	vfprintf(g_log, fmt, ap);
	va_end(ap);

	fputc('\n', g_log);
	fflush(g_log);
}

static void signal_handler(int signo)
{
	(void)signo;
	g_shutdown = 1;
}

static void usage(const char *program)
{
	fprintf(stderr, "Usage: %s [-p port]\n", program);
}

static int parse_port(const char *text)
{
	char *end;
	long port;

	errno = 0;
	port = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || port <= 0 || port > 65535)
		return -1;
	return (int)port;
}

static int mkdir_p(const char *path)
{
	char tmp[PATH_MAX];
	char *p;

	snprintf(tmp, sizeof(tmp), "%s", path);
	for (p = tmp + 1; *p; ++p) {
		if (*p == '/') {
			*p = '\0';
			if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
				return -1;
			*p = '/';
		}
	}

	if (mkdir(tmp, 0755) != 0 && errno != EEXIST)
		return -1;
	return 0;
}

static int init_dirs(void)
{
	if (mkdir_p(APP_DIR) != 0)
		return -1;
	if (mkdir_p(BIN_DIR) != 0)
		return -1;
	if (mkdir_p(RECORDINGS_DIR) != 0)
		return -1;
	if (mkdir_p(LOG_DIR) != 0)
		return -1;
	return 0;
}

static int init_log(void)
{
	time_t now = time(NULL);
	struct tm tmv;
	char name[PATH_MAX];
	char link_path[PATH_MAX];
	char rel[NAME_MAX + 32];

	localtime_r(&now, &tmv);
	strftime(rel, sizeof(rel), "camera_server_%Y-%m-%d_%H-%M-%S.log", &tmv);
	snprintf(name, sizeof(name), "%s/%s", LOG_DIR, rel);
	g_log = fopen(name, "a");
	if (g_log == NULL)
		return -1;

	snprintf(link_path, sizeof(link_path), "%s/latest.log", LOG_DIR);
	unlink(link_path);
	symlink(rel, link_path);
	log_msg("server start");
	return 0;
}

static bool safe_filename(const char *name)
{
	if (name == NULL || *name == '\0' || strlen(name) > NAME_MAX)
		return false;
	if (strstr(name, "..") != NULL || strchr(name, '/') != NULL)
		return false;
	for (const unsigned char *p = (const unsigned char *)name; *p; ++p) {
		if (!(isalnum(*p) || *p == '.' || *p == '_' || *p == '-'))
			return false;
	}
	return true;
}

static bool has_suffix(const char *text, const char *suffix)
{
	size_t text_len = strlen(text);
	size_t suffix_len = strlen(suffix);

	return text_len >= suffix_len &&
		strcmp(text + text_len - suffix_len, suffix) == 0;
}

static uint64_t free_bytes(void)
{
	struct statvfs st;

	if (statvfs(RECORDINGS_DIR, &st) != 0)
		return 0;
	return (uint64_t)st.f_bavail * (uint64_t)st.f_frsize;
}

static bool low_storage(void)
{
	return free_bytes() <= LOW_SPACE_BYTES;
}

static void make_datetime_name(char *buf, size_t size, const char *prefix, const char *suffix)
{
	time_t now = time(NULL);
	struct tm tmv;
	char ts[32];

	localtime_r(&now, &tmv);
	strftime(ts, sizeof(ts), "%Y-%m-%d_%H-%M-%S", &tmv);
	snprintf(buf, size, "%s%s%s", prefix, ts, suffix);
}

static void find_executable(char *out, size_t size, const char *system_path,
	const char *legacy_path, const char *local_name)
{
	if (access(system_path, X_OK) == 0) {
		snprintf(out, size, "%s", system_path);
	} else if (access(legacy_path, X_OK) == 0) {
		snprintf(out, size, "%s", legacy_path);
	} else {
		snprintf(out, size, "%s", local_name);
	}
}

static const char *state_name(rec_state_t state)
{
	switch (state) {
	case REC_IDLE: return "idle";
	case REC_STARTING: return "starting";
	case REC_RECORDING: return "recording";
	case REC_STOPPING: return "stopping";
	case REC_ERROR: return "error";
	default: return "unknown";
	}
}

static const char *clip_state_name(clip_state_t state)
{
	switch (state) {
	case CLIP_NONE: return "none";
	case CLIP_RUNNING: return "running";
	case CLIP_DONE: return "done";
	case CLIP_FAILED: return "failed";
	default: return "unknown";
	}
}

static void poll_children(void)
{
	int status;
	pid_t pid;

	while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
		if (pid == g_rec_pid) {
			if (g_rec_state == REC_STOPPING || (WIFEXITED(status) && WEXITSTATUS(status) == 0)) {
				log_msg("record stop success: %s", g_current_file);
				g_rec_state = REC_IDLE;
				set_last_error("");
			} else {
				log_msg("record process failed: status=%d file=%s", status, g_current_file);
				g_rec_state = REC_ERROR;
				set_last_error("recording process failed");
			}
			g_rec_pid = -1;
		} else if (pid == g_clip_pid) {
			g_clip_finished_at = time(NULL);
			if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
				g_clip_state = CLIP_DONE;
				g_clip_error[0] = '\0';
				log_msg("clip success: %s -> %s", g_clip_input, g_clip_output);
			} else {
				g_clip_state = CLIP_FAILED;
				snprintf(g_clip_error, sizeof(g_clip_error), "camera_clip failed: status=%d", status);
				log_msg("clip failed: %s", g_clip_error);
			}
			g_clip_pid = -1;
		}
	}
}

static void finish_recording_child(int status)
{
	if (g_rec_state == REC_STOPPING || (WIFEXITED(status) && WEXITSTATUS(status) == 0)) {
		log_msg("record stop success: %s", g_current_file);
		g_rec_state = REC_IDLE;
		set_last_error("");
	} else {
		log_msg("record process failed: status=%d file=%s", status, g_current_file);
		g_rec_state = REC_ERROR;
		set_last_error("recording process failed");
	}
	g_rec_pid = -1;
}

static int wait_recording_stopped(pid_t pid, int timeout_ms)
{
	int waited_ms = 0;
	int status;
	pid_t rc;

	while (waited_ms <= timeout_ms) {
		rc = waitpid(pid, &status, WNOHANG);
		if (rc == pid) {
			finish_recording_child(status);
			return 0;
		}
		if (rc < 0) {
			if (errno == ECHILD) {
				g_rec_pid = -1;
				g_rec_state = REC_IDLE;
				set_last_error("");
				return 0;
			}
			return -1;
		}
		usleep(100 * 1000);
		waited_ms += 100;
	}

	return 1;
}

static int start_recording(void)
{
	char name[NAME_MAX + 1];
	char path[PATH_MAX];
	char exe[PATH_MAX];
	pid_t pid;
	pid_t existing;

	poll_children();
	if (g_rec_state == REC_STARTING || g_rec_state == REC_RECORDING || g_rec_state == REC_STOPPING) {
		set_last_error("recording already active");
		return 409;
	}
	existing = find_camera_stream_process();
	if (existing > 0) {
		snprintf(g_last_error, sizeof(g_last_error),
			"camera_stream already running: pid=%ld", (long)existing);
		log_msg("record start rejected: %s", g_last_error);
		return 409;
	}
	if (low_storage()) {
		set_last_error("low storage");
		log_msg("record start rejected: low storage");
		return 507;
	}

	make_datetime_name(name, sizeof(name), "rec_", ".mp4");
	snprintf(path, sizeof(path), "%s/%s", RECORDINGS_DIR, name);
	find_executable(exe, sizeof(exe),
		SYSTEM_BIN_DIR "/camera-stream",
		BIN_DIR "/camera_stream",
		"./camera_stream");

	g_rec_state = REC_STARTING;
	snprintf(g_current_file, sizeof(g_current_file), "%s", name);
	g_started_at = time(NULL);
	log_msg("record start requested: %s", name);

	pid = fork();
	if (pid < 0) {
		g_rec_state = REC_ERROR;
		set_last_error("fork failed");
		log_msg("record start failed: fork: %s", strerror(errno));
		return 500;
	}
	if (pid == 0) {
		char seconds[32];
		snprintf(seconds, sizeof(seconds), "%d", RECORD_SECONDS);
		execl(exe, exe, "-s", seconds, "-o", path, (char *)NULL);
		_exit(127);
	}

	g_rec_pid = pid;
	g_rec_state = REC_RECORDING;
	set_last_error("");
	log_msg("record start success: pid=%ld file=%s", (long)pid, name);
	return 200;
}

static int stop_recording(void)
{
	pid_t pid;
	int rc;

	poll_children();
	if (g_rec_pid <= 0 || !(g_rec_state == REC_RECORDING || g_rec_state == REC_STARTING)) {
		set_last_error("not recording");
		return 409;
	}

	pid = g_rec_pid;
	g_rec_state = REC_STOPPING;
	log_msg("record stop requested: pid=%ld file=%s", (long)pid, g_current_file);
	if (kill(pid, SIGTERM) != 0) {
		if (errno == ESRCH) {
			g_rec_pid = -1;
			g_rec_state = REC_IDLE;
			set_last_error("");
			log_msg("record stop: recorder already exited");
			return 200;
		}
		g_rec_state = REC_ERROR;
		set_last_error("failed to signal recorder");
		log_msg("record stop failed: %s", strerror(errno));
		return 500;
	}

	rc = wait_recording_stopped(pid, 3000);
	if (rc < 0) {
		g_rec_state = REC_ERROR;
		set_last_error("failed to wait recorder");
		log_msg("record stop wait failed: %s", strerror(errno));
		return 500;
	}
	if (rc > 0) {
		set_last_error("recorder is stopping");
		log_msg("record stop still pending: pid=%ld", (long)pid);
	}

	return 200;
}

static void check_low_space_stop(void)
{
	if ((g_rec_state == REC_RECORDING || g_rec_state == REC_STARTING) && low_storage()) {
		log_msg("low storage reached; stopping recording");
		set_last_error("low storage");
		stop_recording();
	}
}

static bool pid_is_stale_camera_process(pid_t pid)
{
	char path[64];
	char buf[PATH_MAX];
	int fd;
	ssize_t n;

	snprintf(path, sizeof(path), "/proc/%ld/comm", (long)pid);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return false;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return false;
	buf[n] = '\0';
	if (strchr(buf, '\n') != NULL)
		*strchr(buf, '\n') = '\0';
	if (strcmp(buf, "camera_server") != 0 && strcmp(buf, "camera_stream") != 0)
		return false;

	snprintf(path, sizeof(path), "/proc/%ld/cmdline", (long)pid);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return false;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return false;
	buf[n] = '\0';

	for (ssize_t i = 0; i < n; ++i) {
		if (buf[i] == '\0')
			buf[i] = ' ';
	}
	return strstr(buf, "camera_server") != NULL || strstr(buf, "camera_stream") != NULL;
}

static bool pid_is_camera_stream(pid_t pid)
{
	char path[64];
	char buf[PATH_MAX];
	int fd;
	ssize_t n;

	snprintf(path, sizeof(path), "/proc/%ld/comm", (long)pid);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return false;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return false;
	buf[n] = '\0';
	if (strchr(buf, '\n') != NULL)
		*strchr(buf, '\n') = '\0';
	if (strcmp(buf, "camera_stream") != 0)
		return false;

	snprintf(path, sizeof(path), "/proc/%ld/cmdline", (long)pid);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return false;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return false;
	buf[n] = '\0';

	for (ssize_t i = 0; i < n; ++i) {
		if (buf[i] == '\0')
			buf[i] = ' ';
	}
	return strstr(buf, "camera_stream") != NULL;
}

static pid_t find_camera_stream_process(void)
{
	DIR *dir;
	struct dirent *ent;

	dir = opendir("/proc");
	if (dir == NULL)
		return -1;

	while ((ent = readdir(dir)) != NULL) {
		char *end;
		long value;
		pid_t pid;

		errno = 0;
		value = strtol(ent->d_name, &end, 10);
		if (errno != 0 || end == ent->d_name || *end != '\0' || value <= 0)
			continue;

		pid = (pid_t)value;
		if (pid == getpid() || pid == g_rec_pid)
			continue;
		if (pid_is_camera_stream(pid)) {
			closedir(dir);
			return pid;
		}
	}

	closedir(dir);
	return -1;
}

static bool wait_process_gone(pid_t pid, int timeout_ms)
{
	int waited_ms = 0;

	while (waited_ms <= timeout_ms) {
		if (kill(pid, 0) != 0 && errno == ESRCH)
			return true;
		usleep(100 * 1000);
		waited_ms += 100;
	}

	return false;
}

static void check_orphan_recorder_on_startup(void)
{
	pid_t pid = find_camera_stream_process();

	if (pid <= 0)
		return;

	log_msg("found existing camera_stream pid=%ld; requesting graceful stop", (long)pid);
	if (kill(pid, SIGTERM) == 0 && wait_process_gone(pid, 5000)) {
		log_msg("existing camera_stream stopped");
		return;
	}

	g_rec_state = REC_ERROR;
	snprintf(g_last_error, sizeof(g_last_error),
		"existing camera_stream process is still running: pid=%ld", (long)pid);
	log_msg("%s", g_last_error);
}

static unsigned long find_listen_inode_for_port(int port)
{
	char line[512];
	FILE *fp;
	unsigned int local_port;
	char state[8];
	unsigned long inode;

	fp = fopen("/proc/net/tcp", "r");
	if (fp == NULL)
		return 0;

	if (fgets(line, sizeof(line), fp) == NULL) {
		fclose(fp);
		return 0;
	}

	while (fgets(line, sizeof(line), fp) != NULL) {
		local_port = 0;
		state[0] = '\0';
		inode = 0;
		if (sscanf(line, " %*d: %*8[0-9A-Fa-f]:%x %*8[0-9A-Fa-f]:%*x %2s %*s %*s %*s %*s %*s %lu",
		    &local_port, state, &inode) == 3) {
			if ((int)local_port == port && strcmp(state, "0A") == 0) {
				fclose(fp);
				return inode;
			}
		}
	}

	fclose(fp);
	return 0;
}

static bool proc_has_socket_inode(pid_t pid, unsigned long inode)
{
	char fd_dir_path[64];
	char link_path[PATH_MAX];
	char target[128];
	char expected[64];
	DIR *dir;
	struct dirent *ent;

	snprintf(fd_dir_path, sizeof(fd_dir_path), "/proc/%ld/fd", (long)pid);
	dir = opendir(fd_dir_path);
	if (dir == NULL)
		return false;

	snprintf(expected, sizeof(expected), "socket:[%lu]", inode);
	while ((ent = readdir(dir)) != NULL) {
		ssize_t n;

		if (ent->d_name[0] == '.')
			continue;

		snprintf(link_path, sizeof(link_path), "%s/%s", fd_dir_path, ent->d_name);
		n = readlink(link_path, target, sizeof(target) - 1);
		if (n <= 0)
			continue;
		target[n] = '\0';
		if (strcmp(target, expected) == 0) {
			closedir(dir);
			return true;
		}
	}

	closedir(dir);
	return false;
}

static int kill_old_camera_processes_on_port(int port)
{
	DIR *dir;
	struct dirent *ent;
	int killed = 0;
	unsigned long listen_inode;

	listen_inode = find_listen_inode_for_port(port);
	if (listen_inode == 0)
		return 0;

	dir = opendir("/proc");
	if (dir == NULL)
		return 0;

	while ((ent = readdir(dir)) != NULL) {
		char *end;
		long value;
		pid_t pid;

		errno = 0;
		value = strtol(ent->d_name, &end, 10);
		if (errno != 0 || end == ent->d_name || *end != '\0' || value <= 0)
			continue;

		pid = (pid_t)value;
		if (pid == getpid())
			continue;
		if (!pid_is_stale_camera_process(pid))
			continue;
		if (!proc_has_socket_inode(pid, listen_inode))
			continue;

		log_msg("killing old camera process pid=%ld on port %d", (long)pid, port);
		if (kill(pid, SIGTERM) == 0)
			++killed;
	}

	closedir(dir);
	return killed;
}

static int bind_server_socket(int server_fd, struct sockaddr_in *addr)
{
	for (int attempt = 0; attempt < 2; ++attempt) {
		if (bind(server_fd, (struct sockaddr *)addr, sizeof(*addr)) == 0)
			return 0;

		if (errno != EADDRINUSE || attempt != 0)
			return -1;

		fprintf(stderr, "port %d is in use; trying to stop old camera process\n", g_server_port);
		log_msg("port %d is in use; trying to stop old camera process", g_server_port);
		if (kill_old_camera_processes_on_port(g_server_port) <= 0) {
			errno = EADDRINUSE;
			return -1;
		}

		for (int i = 0; i < 20; ++i) {
			usleep(100 * 1000);
			if (bind(server_fd, (struct sockaddr *)addr, sizeof(*addr)) == 0)
				return 0;
			if (errno != EADDRINUSE)
				return -1;
		}
	}

	return -1;
}

static int recording_compare(const void *a, const void *b)
{
	const recording_t *ra = a;
	const recording_t *rb = b;
	if (ra->mtime < rb->mtime)
		return 1;
	if (ra->mtime > rb->mtime)
		return -1;
	return strcmp(ra->name, rb->name);
}

static int list_recordings(recording_t *items, int max_items)
{
	DIR *dir;
	struct dirent *ent;
	int count = 0;

	dir = opendir(RECORDINGS_DIR);
	if (dir == NULL)
		return 0;

	while ((ent = readdir(dir)) != NULL && count < max_items) {
		char path[PATH_MAX];
		struct stat st;

		if (!safe_filename(ent->d_name))
			continue;
		if (!has_suffix(ent->d_name, ".mp4"))
			continue;
		snprintf(path, sizeof(path), "%s/%s", RECORDINGS_DIR, ent->d_name);
		if (stat(path, &st) != 0 || !S_ISREG(st.st_mode))
			continue;

		snprintf(items[count].name, sizeof(items[count].name), "%s", ent->d_name);
		items[count].size = (uint64_t)st.st_size;
		items[count].mtime = st.st_mtime;
		++count;
	}
	closedir(dir);
	qsort(items, count, sizeof(items[0]), recording_compare);
	return count;
}

static void send_all(int fd, const char *buf, size_t len)
{
	while (len > 0) {
		ssize_t n = send(fd, buf, len, MSG_NOSIGNAL);
		if (n <= 0)
			return;
		buf += n;
		len -= (size_t)n;
	}
}

static void set_socket_timeouts(int fd)
{
	struct timeval tv;

	tv.tv_sec = 5;
	tv.tv_usec = 0;
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void http_reply(int fd, int code, const char *type, const char *body)
{
	char head[512];
	const char *status = "OK";
	size_t len = body != NULL ? strlen(body) : 0;

	if (code == 400) status = "Bad Request";
	else if (code == 404) status = "Not Found";
	else if (code == 405) status = "Method Not Allowed";
	else if (code == 409) status = "Conflict";
	else if (code == 500) status = "Internal Server Error";
	else if (code == 507) status = "Insufficient Storage";

	snprintf(head, sizeof(head),
		"HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n",
		code, status, type, len);
	send_all(fd, head, strlen(head));
	if (len > 0)
		send_all(fd, body, len);
}

static void json_escape(char *dst, size_t size, const char *src)
{
	size_t pos = 0;
	for (const unsigned char *p = (const unsigned char *)src; *p && pos + 2 < size; ++p) {
		if (*p == '"' || *p == '\\') {
			if (pos + 3 >= size)
				break;
			dst[pos++] = '\\';
			dst[pos++] = (char)*p;
		} else if (*p >= 0x20) {
			dst[pos++] = (char)*p;
		}
	}
	dst[pos] = '\0';
}

static void reply_status(int fd)
{
	char body[2048];
	char file[PATH_MAX * 2];
	char err[512];
	time_t now = time(NULL);
	long duration = 0;

	poll_children();
	check_low_space_stop();
	if (g_started_at > 0 && (g_rec_state == REC_RECORDING || g_rec_state == REC_STOPPING))
		duration = (long)(now - g_started_at) * 1000L;
	json_escape(file, sizeof(file), g_current_file);
	json_escape(err, sizeof(err), g_last_error);
	snprintf(body, sizeof(body),
		"{\"state\":\"%s\",\"recording\":%s,\"current_file\":\"%s\","
		"\"duration_ms\":%ld,\"free_bytes\":%llu,\"low_storage\":%s,"
		"\"last_error\":\"%s\"}",
		state_name(g_rec_state),
		g_rec_state == REC_RECORDING ? "true" : "false",
		file, duration, (unsigned long long)free_bytes(),
		low_storage() ? "true" : "false", err);
	http_reply(fd, 200, "application/json", body);
}

static void reply_recordings(int fd)
{
	recording_t items[MAX_FILES];
	char body[32768];
	char *p = body;
	size_t left = sizeof(body);
	int count = list_recordings(items, MAX_FILES);

	p += snprintf(p, left, "[");
	left = sizeof(body) - (size_t)(p - body);
	for (int i = 0; i < count && left > 128; ++i) {
		char name[NAME_MAX * 2];
		json_escape(name, sizeof(name), items[i].name);
		p += snprintf(p, left, "%s{\"name\":\"%s\",\"size\":%llu,\"mtime\":%lld}",
			i == 0 ? "" : ",", name, (unsigned long long)items[i].size,
			(long long)items[i].mtime);
		left = sizeof(body) - (size_t)(p - body);
	}
	snprintf(p, left, "]");
	http_reply(fd, 200, "application/json", body);
}

static void reply_clip_task(int fd)
{
	char body[2048];
	char input[NAME_MAX * 2];
	char output[NAME_MAX * 2];
	char err[512];

	poll_children();
	json_escape(input, sizeof(input), g_clip_input);
	json_escape(output, sizeof(output), g_clip_output);
	json_escape(err, sizeof(err), g_clip_error);
	snprintf(body, sizeof(body),
		"{\"state\":\"%s\",\"input\":\"%s\",\"output\":\"%s\",\"error\":\"%s\","
		"\"started_at\":%lld,\"finished_at\":%lld}",
		clip_state_name(g_clip_state), input, output, err,
		(long long)g_clip_started_at, (long long)g_clip_finished_at);
	http_reply(fd, 200, "application/json", body);
}

static bool parse_json_long(const char *body, const char *key, long *value)
{
	const char *p = strstr(body, key);
	char *end;
	if (p == NULL)
		return false;
	p = strchr(p, ':');
	if (p == NULL)
		return false;
	++p;
	while (*p == ' ' || *p == '\t')
		++p;
	errno = 0;
	*value = strtol(p, &end, 10);
	return errno == 0 && end != p;
}

static bool parse_json_audio(const char *body)
{
	const char *p = strstr(body, "\"audio\"");
	if (p == NULL)
		return true;
	p = strchr(p, ':');
	if (p == NULL)
		return true;
	++p;
	while (*p == ' ' || *p == '\t')
		++p;
	return strncmp(p, "false", 5) != 0;
}

static long parse_content_length(const char *req)
{
	const char *p = strstr(req, "Content-Length:");
	char *end;

	if (p == NULL)
		p = strstr(req, "content-length:");
	if (p == NULL)
		return 0;

	p = strchr(p, ':');
	if (p == NULL)
		return 0;
	++p;
	while (*p == ' ' || *p == '\t')
		++p;

	errno = 0;
	long value = strtol(p, &end, 10);
	if (errno != 0 || end == p || value < 0 || value > MAX_BODY)
		return 0;
	return value;
}

static void make_clip_output(char *out, size_t size, const char *input, long start_ms, long end_ms, bool audio)
{
	char base[NAME_MAX + 1];
	char *dot;
	long ss = start_ms / 1000;
	long es = end_ms / 1000;

	snprintf(base, sizeof(base), "%s", input);
	dot = strrchr(base, '.');
	if (dot != NULL)
		*dot = '\0';
	snprintf(out, size, "%s_%02ld-%02ld-%02ld_%02ld-%02ld-%02ld%s.mp4",
		base, ss / 3600, ss / 60 % 60, ss % 60,
		es / 3600, es / 60 % 60, es % 60,
		audio ? "" : "_noaudio");
}

static int start_clip(const char *filename, const char *body)
{
	long start_ms;
	long end_ms;
	bool audio;
	char input_path[PATH_MAX];
	char output_path[PATH_MAX];
	char exe[PATH_MAX];
	pid_t pid;

	poll_children();
	if (g_clip_state == CLIP_RUNNING)
		return 409;
	if (!safe_filename(filename))
		return 400;
	if (!parse_json_long(body, "\"start_ms\"", &start_ms) ||
	    !parse_json_long(body, "\"end_ms\"", &end_ms) ||
	    start_ms < 0 || end_ms <= start_ms)
		return 400;
	if (low_storage()) {
		log_msg("clip rejected: low storage");
		return 507;
	}

	audio = parse_json_audio(body);
	snprintf(g_clip_input, sizeof(g_clip_input), "%s", filename);
	make_clip_output(g_clip_output, sizeof(g_clip_output), filename, start_ms, end_ms, audio);
	snprintf(input_path, sizeof(input_path), "%s/%s", RECORDINGS_DIR, g_clip_input);
	snprintf(output_path, sizeof(output_path), "%s/%s", RECORDINGS_DIR, g_clip_output);
	find_executable(exe, sizeof(exe),
		LIBEXEC_DIR "/camera_clip",
		BIN_DIR "/camera_clip",
		"./camera_clip");

	g_clip_state = CLIP_RUNNING;
	g_clip_error[0] = '\0';
	g_clip_started_at = time(NULL);
	g_clip_finished_at = 0;
	log_msg("clip accepted: %s -> %s", g_clip_input, g_clip_output);

	pid = fork();
	if (pid < 0) {
		g_clip_state = CLIP_FAILED;
		snprintf(g_clip_error, sizeof(g_clip_error), "fork failed");
		return 500;
	}
	if (pid == 0) {
		char start_arg[32];
		char end_arg[32];
		snprintf(start_arg, sizeof(start_arg), "%ld", start_ms);
		snprintf(end_arg, sizeof(end_arg), "%ld", end_ms);
		if (audio)
			execl(exe, exe, input_path, output_path, "--start-ms", start_arg, "--end-ms", end_arg, (char *)NULL);
		else
			execl(exe, exe, input_path, output_path, "--start-ms", start_arg, "--end-ms", end_arg, "--no-audio", (char *)NULL);
		_exit(127);
	}
	g_clip_pid = pid;
	return 200;
}

static bool parse_range_header(const char *req, off_t file_size, off_t *start, off_t *end)
{
	const char *p = strstr(req, "Range:");
	char *dash;
	char *parse_end;
	long long a;
	long long b;

	if (p == NULL)
		p = strstr(req, "range:");
	if (p == NULL)
		return false;

	p = strstr(p, "bytes=");
	if (p == NULL)
		return false;
	p += 6;

	dash = strchr(p, '-');
	if (dash == NULL)
		return false;

	if (dash == p) {
		errno = 0;
		b = strtoll(dash + 1, &parse_end, 10);
		if (errno != 0 || parse_end == dash + 1 || b <= 0)
			return false;
		if (b > file_size)
			b = file_size;
		*start = file_size - (off_t)b;
		*end = file_size - 1;
		return true;
	}

	errno = 0;
	a = strtoll(p, &parse_end, 10);
	if (errno != 0 || parse_end != dash || a < 0)
		return false;

	if (*(dash + 1) == '\0' || *(dash + 1) == '\r' || *(dash + 1) == '\n' || *(dash + 1) == ' ') {
		b = file_size - 1;
	} else {
		errno = 0;
		b = strtoll(dash + 1, &parse_end, 10);
		if (errno != 0 || parse_end == dash + 1 || b < a)
			return false;
	}

	*start = (off_t)a;
	*end = b >= file_size ? file_size - 1 : (off_t)b;
	return true;
}

static void send_file_range_not_satisfiable(int fd, off_t file_size)
{
	char head[256];

	snprintf(head, sizeof(head),
		"HTTP/1.1 416 Range Not Satisfiable\r\n"
		"Content-Range: bytes */%lld\r\n"
		"Content-Length: 0\r\n"
		"Connection: close\r\n\r\n",
		(long long)file_size);
	send_all(fd, head, strlen(head));
}

static void send_file(int fd, const char *filename, const char *req)
{
	char path[PATH_MAX];
	char head[768];
	struct stat st;
	int in;
	char buf[8192];
	off_t start = 0;
	off_t end;
	off_t remaining;
	bool partial = false;
	ssize_t n;

	if (!safe_filename(filename)) {
		http_reply(fd, 400, "application/json", "{\"error\":\"bad filename\"}");
		return;
	}
	snprintf(path, sizeof(path), "%s/%s", RECORDINGS_DIR, filename);
	in = open(path, O_RDONLY);
	if (in < 0 || fstat(in, &st) != 0 || !S_ISREG(st.st_mode)) {
		if (in >= 0)
			close(in);
		http_reply(fd, 404, "application/json", "{\"error\":\"not found\"}");
		return;
	}

	if (st.st_size <= 0) {
		close(in);
		http_reply(fd, 404, "application/json", "{\"error\":\"empty file\"}");
		return;
	}

	end = st.st_size - 1;
	if (parse_range_header(req, st.st_size, &start, &end)) {
		if (start < 0 || start >= st.st_size || end < start) {
			send_file_range_not_satisfiable(fd, st.st_size);
			close(in);
			return;
		}
		partial = true;
	}

	if (lseek(in, start, SEEK_SET) < 0) {
		close(in);
		http_reply(fd, 500, "application/json", "{\"error\":\"seek failed\"}");
		return;
	}

	remaining = end - start + 1;
	if (partial) {
		snprintf(head, sizeof(head),
			"HTTP/1.1 206 Partial Content\r\nContent-Type: video/mp4\r\n"
			"Content-Length: %lld\r\nContent-Range: bytes %lld-%lld/%lld\r\n"
			"Content-Disposition: inline; filename=\"%s\"\r\nAccept-Ranges: bytes\r\nConnection: close\r\n\r\n",
			(long long)remaining, (long long)start, (long long)end,
			(long long)st.st_size, filename);
	} else {
		snprintf(head, sizeof(head),
			"HTTP/1.1 200 OK\r\nContent-Type: video/mp4\r\nContent-Length: %lld\r\n"
			"Content-Disposition: inline; filename=\"%s\"\r\nAccept-Ranges: bytes\r\nConnection: close\r\n\r\n",
			(long long)remaining, filename);
	}
	send_all(fd, head, strlen(head));

	while (remaining > 0) {
		size_t want = remaining > (off_t)sizeof(buf) ? sizeof(buf) : (size_t)remaining;
		n = read(in, buf, want);
		if (n <= 0)
			break;
		send_all(fd, buf, (size_t)n);
		remaining -= n;
	}
	close(in);
}

static void send_file_in_child(int fd, const char *filename, const char *req)
{
	pid_t pid = fork();

	if (pid < 0) {
		http_reply(fd, 500, "application/json", "{\"error\":\"fork failed\"}");
		return;
	}

	if (pid == 0) {
		if (g_server_fd >= 0)
			close(g_server_fd);
		set_socket_timeouts(fd);
		send_file(fd, filename, req);
		close(fd);
		_exit(0);
	}
}

static const char *index_html =
"<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Camera Recorder</title><style>"
"body{font-family:sans-serif;margin:20px;line-height:1.4}button{margin:2px 6px 2px 0}table{border-collapse:collapse;width:100%;max-width:1100px}"
"td,th{border-bottom:1px solid #ccc;padding:4px 6px;text-align:left}pre{background:#f5f5f5;padding:8px;overflow:auto}.small{color:#555}"
"</style></head><body><h1>Camera Recorder</h1>"
"<h2>Status</h2><pre id=s>loading...</pre>"
"<p><button onclick=startRec()>Start Recording</button><button onclick=stopRec()>Stop Recording</button></p>"
"<h2>Recordings</h2><table><thead><tr><th>Name</th><th>Size</th><th>Modified</th><th>Actions</th></tr></thead><tbody id=r></tbody></table>"
"<h2>Clip Task</h2><pre id=t>loading...</pre>"
"<p class=small>Clip input uses seconds. Playback is via the file links.</p>"
"<script>"
"async function api(u,o){let r=await fetch(u,o);let tx=await r.text();try{return JSON.parse(tx)}catch(e){return tx}}"
"function size(n){return(n/1048576).toFixed(1)+' MiB'}"
"function ts(v){return v?new Date(v*1000).toLocaleString():''}"
"async function refresh(){let s=await api('/api/status');document.getElementById('s').textContent=JSON.stringify(s,null,2);"
"let rs=await api('/api/recordings');document.getElementById('r').innerHTML=rs.map(x=>`<tr><td><a href=\"/recordings/${x.name}\">${x.name}</a></td><td>${size(x.size)}</td><td>${ts(x.mtime)}</td><td><button onclick=\"clip('${x.name}')\">Clip</button><button onclick=\"del('${x.name}')\">Delete</button></td></tr>`).join('');"
"let t=await api('/api/clip/task');document.getElementById('t').textContent=JSON.stringify(t,null,2)}"
"async function startRec(){await fetch('/api/record/start',{method:'POST'});refresh()}async function stopRec(){if(confirm('Stop recording?')){await fetch('/api/record/stop',{method:'POST'});refresh()}}"
"async function del(n){if(confirm('Delete '+n+'?')){await fetch('/api/recordings/'+n,{method:'DELETE'});refresh()}}"
"async function clip(n){let s=prompt('Start seconds','0');if(s==null)return;let e=prompt('End seconds','5');if(e==null)return;let a=confirm('Include audio? OK=yes Cancel=no');let r=await fetch('/api/recordings/'+n+'/clip',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({start_ms:Math.round(+s*1000),end_ms:Math.round(+e*1000),audio:a})});if(!r.ok)alert(await r.text());refresh()}"
"setInterval(refresh,1000);refresh();</script></body></html>";

static void handle_client(int fd)
{
	char req[MAX_REQ + 1];
	char method[8];
	char path[PATH_MAX];
	char *body;
	ssize_t n;
	long content_length;
	size_t header_bytes;
	size_t have_body;

	n = recv(fd, req, MAX_REQ, 0);
	if (n <= 0)
		return;
	req[n] = '\0';
	body = strstr(req, "\r\n\r\n");
	if (body != NULL) {
		body += 4;
		content_length = parse_content_length(req);
		header_bytes = (size_t)(body - req);
		have_body = (size_t)n > header_bytes ? (size_t)n - header_bytes : 0;
		while (have_body < (size_t)content_length && (size_t)n < MAX_REQ) {
			ssize_t more = recv(fd, req + n, MAX_REQ - (size_t)n, 0);
			if (more <= 0)
				break;
			n += more;
			req[n] = '\0';
			body = req + header_bytes;
			have_body = (size_t)n - header_bytes;
		}
		if (content_length > 0 && have_body < (size_t)content_length) {
			http_reply(fd, 400, "application/json", "{\"error\":\"short body\"}");
			return;
		}
	} else {
		body = "";
	}

	if (sscanf(req, "%7s %1023s", method, path) != 2) {
		http_reply(fd, 400, "application/json", "{\"error\":\"bad request\"}");
		return;
	}

	if (strcmp(method, "GET") == 0 && strcmp(path, "/") == 0) {
		http_reply(fd, 200, "text/html; charset=utf-8", index_html);
	} else if (strcmp(method, "GET") == 0 && strcmp(path, "/api/status") == 0) {
		reply_status(fd);
	} else if (strcmp(method, "POST") == 0 && strcmp(path, "/api/record/start") == 0) {
		int rc = start_recording();
		http_reply(fd, rc, "application/json", rc == 200 ? "{\"ok\":true}" : "{\"ok\":false}");
	} else if (strcmp(method, "POST") == 0 && strcmp(path, "/api/record/stop") == 0) {
		int rc = stop_recording();
		http_reply(fd, rc, "application/json", rc == 200 ? "{\"ok\":true}" : "{\"ok\":false}");
	} else if (strcmp(method, "GET") == 0 && strcmp(path, "/api/recordings") == 0) {
		reply_recordings(fd);
	} else if (strcmp(method, "GET") == 0 && strncmp(path, "/recordings/", 12) == 0) {
		send_file_in_child(fd, path + 12, req);
	} else if (strcmp(method, "DELETE") == 0 && strncmp(path, "/api/recordings/", 16) == 0) {
		char full[PATH_MAX];
		const char *name = path + 16;
		if (!safe_filename(name)) {
			http_reply(fd, 400, "application/json", "{\"error\":\"bad filename\"}");
		} else {
			snprintf(full, sizeof(full), "%s/%s", RECORDINGS_DIR, name);
			if (unlink(full) == 0) {
				log_msg("recording deleted: %s", name);
				http_reply(fd, 200, "application/json", "{\"ok\":true}");
			} else {
				http_reply(fd, 404, "application/json", "{\"error\":\"not found\"}");
			}
		}
	} else if (strcmp(method, "POST") == 0 && strncmp(path, "/api/recordings/", 16) == 0) {
		char *clip = strstr(path + 16, "/clip");
		if (clip != NULL && strcmp(clip, "/clip") == 0) {
			char name[NAME_MAX + 1];
			size_t len = (size_t)(clip - (path + 16));
			if (len >= sizeof(name))
				len = sizeof(name) - 1;
			memcpy(name, path + 16, len);
			name[len] = '\0';
			int rc = start_clip(name, body);
			http_reply(fd, rc, "application/json", rc == 200 ? "{\"ok\":true}" : "{\"ok\":false}");
		} else {
			http_reply(fd, 404, "application/json", "{\"error\":\"not found\"}");
		}
	} else if (strcmp(method, "GET") == 0 && strcmp(path, "/api/clip/task") == 0) {
		reply_clip_task(fd);
	} else {
		http_reply(fd, 404, "application/json", "{\"error\":\"not found\"}");
	}
}

int main(int argc, char **argv)
{
	int server_fd;
	int yes = 1;
	struct sockaddr_in addr;
	int opt;

	while ((opt = getopt(argc, argv, "p:")) != -1) {
		switch (opt) {
		case 'p':
			g_server_port = parse_port(optarg);
			if (g_server_port < 0) {
				usage(argv[0]);
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

	if (init_dirs() != 0 || init_log() != 0) {
		fprintf(stderr, "failed to initialize app directories/logs\n");
		return 1;
	}
	check_orphan_recorder_on_startup();

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGPIPE, SIG_IGN);
	g_shutdown = 0;

	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_fd < 0) {
		fprintf(stderr, "socket failed: %s\n", strerror(errno));
		log_msg("socket failed: %s", strerror(errno));
		return 1;
	}
	g_server_fd = server_fd;
	if (fcntl(server_fd, F_SETFD, FD_CLOEXEC) != 0) {
		fprintf(stderr, "fcntl FD_CLOEXEC failed: %s\n", strerror(errno));
		log_msg("fcntl FD_CLOEXEC failed: %s", strerror(errno));
		close(server_fd);
		return 1;
	}
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(g_server_port);

	if (bind_server_socket(server_fd, &addr) != 0) {
		fprintf(stderr, "bind 0.0.0.0:%d failed: %s\n", g_server_port, strerror(errno));
		log_msg("bind 0.0.0.0:%d failed: %s", g_server_port, strerror(errno));
		close(server_fd);
		return 1;
	}
	if (listen(server_fd, 16) != 0) {
		fprintf(stderr, "listen failed: %s\n", strerror(errno));
		log_msg("listen failed: %s", strerror(errno));
		close(server_fd);
		return 1;
	}

	log_msg("listening on port %d", g_server_port);
	while (!g_shutdown) {
		fd_set read_fds;
		struct timeval timeout;
		int ready;
		int client;

		FD_ZERO(&read_fds);
		FD_SET(server_fd, &read_fds);
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		ready = select(server_fd + 1, &read_fds, NULL, NULL, &timeout);
		poll_children();
		check_low_space_stop();
		if (ready < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (ready == 0 || !FD_ISSET(server_fd, &read_fds))
			continue;

		client = accept(server_fd, NULL, NULL);
		if (client < 0)
			continue;
		set_socket_timeouts(client);
		handle_client(client);
		close(client);
		poll_children();
		check_low_space_stop();
	}

	if (g_rec_pid > 0)
		stop_recording();
	if (g_clip_pid > 0)
		kill(g_clip_pid, SIGTERM);
	log_msg("server stop");
	if (g_log != NULL)
		fclose(g_log);
	close(server_fd);
	return 0;
}
